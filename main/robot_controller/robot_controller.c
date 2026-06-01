#include "robot_controller.h"

#include "controller_data/controller_data_config.h"
#include "kinematics/kinematics.h"
#include "mros/mros.h"
#include "mros/mros_param.h"
#include "odrive/odrive.h"
#include "odrive/odrive_types.h"
#include "servo/servo.h"
#include "utils/math_utils.h"
#include "utils/timing_utils.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <math.h>
#include <nav_msgs/msg/odometry.h>
#include <sdkconfig.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <sys/time.h>

static const cmd_t cmd_t_zero = {.timestamp.tv_sec = 0, .timestamp.tv_usec = 0, .linear_vel = 0, .angular_vel = 0, .angle = 0, .correction_active = false};

static odrive_context_t *s_odrive_ml_context;
static odrive_context_t *s_odrive_mr_context;
static servo_t *s_servo_context;
static QueueHandle_t s_cmd_q;

static TaskHandle_t s_base_control_task_h;

static EventGroupHandle_t s_base_control_evt_group;
#define BASE_CONTROL_RUN_BIT BIT0
#define BASE_CONTROL_TERM_BIT BIT1

static EventGroupHandle_t s_base_control_err_evt_group; // This is for external systems to be able to react to an error
static EventBits_t s_base_control_err_bit;

#if ENABLE_CONTROLLER_DATA_PUBLISH == 1
static void new_controller_data(std_msgs__msg__Float64MultiArray *controller_data_msg, wallclock_timestamp_t *timestamp, const double r, const double y, const double u) {
    for (int i = CONTROLLER_DATA_SIZE * 4 - 1; i >= 4; i--) {
        controller_data_msg->data.data[i] = controller_data_msg->data.data[i - 4];
    }
    controller_data_msg->data.data[0] = (double)timestamp->tv_sec + (double)timestamp->tv_usec / 1000000.0;
    controller_data_msg->data.data[1] = r;
    controller_data_msg->data.data[2] = y;
    controller_data_msg->data.data[3] = u;
}
#endif

static void base_control_task(void *pv) {
    ESP_UNUSED(pv);
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Base control task started");

    cmd_t cmd_local;
    sensor_msgs__msg__Imu imu_local;
    float yaw_err, current_yaw, controller, controller_a, controller_b;
    float last_controller = 0.0f;
    float last_err = 0.0f;
    robot_parameters_t params_local;
    float user_suppress, k;
    float correction_on = 0.0f;

    wallclock_timestamp_t time_current;
    wallclock_timestamp_t time_last_cmd_delta;
    long long last_time_cmd_delta_us;

    float torque_ff_ml = 0.0f;
    float torque_ff_mr = 0.0f;
    inverse_kinematics_input_t ik_input;
    inverse_kinematics_output_t ik_output;

#if ENABLE_CONTROLLER_DATA_PUBLISH == 1
    std_msgs__msg__Float64MultiArray controller_data_msg;
    if (mros_init_controller_data_msg(&controller_data_msg) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to initialize controller data message");
        vTaskDelete(NULL);
    }
#endif

    EventBits_t bits = xEventGroupWaitBits(s_base_control_evt_group, BASE_CONTROL_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY); // Wait for start signal

    if (xQueuePeek(s_cmd_q, &cmd_local, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to get initial cmd message");
        vTaskDelete(NULL);
    }

    while (true) {
        bits = xEventGroupGetBits(s_base_control_evt_group);
        if (!(bits & BASE_CONTROL_RUN_BIT)) {
            break; // Instead of terminating every time the run bit is unset, we could just set it to a waiting state -> can be restarted again
        }

        if (xQueuePeek(s_cmd_q, &cmd_local, 0) != pdTRUE) {
            ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to get cmd message");
            break;
        }

        gettimeofday(&time_current, NULL);
        timersub(&time_current, &cmd_local.timestamp, &time_last_cmd_delta);
        last_time_cmd_delta_us = time_last_cmd_delta.tv_sec * 1000000LL + (long long)time_last_cmd_delta.tv_usec;
        // stop if timeout exceeded
        if (last_time_cmd_delta_us > (long long)MS_TO_US(MROS_CMD_VEL_TIMEOUT_MS)) {
            if (odrive_set_velocity(s_odrive_ml_context, 0.0f, torque_ff_ml) != ESP_OK) {
                ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set velocity for ODrive Node ID %d", s_odrive_ml_context->node_id);
                break;
            }
            if (odrive_set_velocity(s_odrive_mr_context, 0.0f, torque_ff_mr) != ESP_OK) {
                ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set velocity for ODrive Node ID %d", s_odrive_mr_context->node_id);
                break;
            }
        } else {
            // normal operation
            if (cmd_local.correction_active == true) {
                // try to get imu msg -> if fail use cmd_vel.twsit for inverse kinematic
                if (mros_peek_imu_msg(&imu_local) != ESP_OK) {
                    ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to peek IMU msg.");
                    last_controller = 0.0f;
                    last_err = 0.0f;
                    ik_input.omega_z = cmd_local.angular_vel;
                } else {
                    if (robot_parameters_get(&params_local) != ESP_OK) {
                        robot_parameters_get_preconfigured(&params_local);
                    }
                    current_yaw = math_normalize_angle(math_quaternion_to_yaw(imu_local.orientation.x, imu_local.orientation.y, imu_local.orientation.z, imu_local.orientation.w));
                    yaw_err = math_normalize_angle(cmd_local.angle - current_yaw);

                    if (params_local.pt2_enable == true) {
                        controller_a = 1 / 2 * params_local.pt2_w * params_local.pt2_w * 0.02;
                        controller_b = 2 * params_local.pt2_D * params_local.pt2_w;
                        controller = CLAMP(last_controller + (controller_a + controller_b) * yaw_err + (controller_a - controller_b) * last_err, -params_local.max_angular_velocity, params_local.max_angular_velocity);
                    } else {
                        controller = CLAMP(params_local.kp * yaw_err, -params_local.max_angular_velocity, params_local.max_angular_velocity);
                    }

                    last_controller = controller;
                    last_err = yaw_err;

                    user_suppress = CLAMP(params_local.max_corner_suppress * (1.0f - (fabsf((float)cmd_local.angular_vel) / params_local.max_angular_velocity)),
                                          0.0f,
                                          params_local.max_corner_suppress); // suppress correction when user is turning
                    k = CLAMP(correction_on * user_suppress * params_local.correction_weight, 0.0f, 1.0f);

                    ik_input.omega_z =
                        CLAMP((float)cmd_local.angular_vel + k * (controller - (float)cmd_local.angular_vel), -params_local.max_angular_velocity, params_local.max_angular_velocity); // k * cntroller + (1 - k) * user
#if ENABLE_CONTROLLER_DATA_PUBLISH == 1
                    new_controller_data(&controller_data_msg, &time_current, cmd_local.angle, current_yaw, ik_input.omega_z);
                    if (mros_update_controller_data(&controller_data_msg) != ESP_OK) {
                        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to update IMU data");
                    }
#endif
                }
            } else {
                last_controller = 0.0f;
                last_err = 0.0f;
                ik_input.omega_z = cmd_local.angular_vel;
            }
            ik_input.vel_x = cmd_local.linear_vel;

            // ik_input.omega_z = ik_input.omega_z + 0.5f; // some error for testing

            if (inverse_kinematics(&ik_input, &ik_output) != ESP_OK) {
                ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to calculate inverse kinematics");
                return;
            }

            if (odrive_set_velocity(s_odrive_ml_context, RAD_TO_REV(ik_output.vel_l), torque_ff_ml) != ESP_OK) {
                ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set velocity for ODrive Node ID %d", s_odrive_ml_context->node_id);
                return;
            }
            if (odrive_set_velocity(s_odrive_mr_context, RAD_TO_REV(ik_output.vel_r), torque_ff_mr) != ESP_OK) {
                ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set velocity for ODrive Node ID %d", s_odrive_mr_context->node_id);
                return;
            }
            if (servo_set_angle(s_servo_context, RAD_TO_DEG(ik_output.steer)) != ESP_OK) {
                ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set steering angle");
                return;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // try to stop the robot
    if (odrive_set_velocity(s_odrive_ml_context, 0.0f, 0.0f) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to stop ODrive Node ID %d", s_odrive_ml_context->node_id);
    }
    if (odrive_set_velocity(s_odrive_mr_context, 0.0f, 0.0f) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to stop ODrive Node ID %d", s_odrive_mr_context->node_id);
    }
    if (servo_set_angle(s_servo_context, 0.0f) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to center steering servo");
    }

    if ((bits & BASE_CONTROL_RUN_BIT)) {
        xEventGroupSetBits(s_base_control_err_evt_group, s_base_control_err_bit);
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Base control task terminated unexpectedly");
    }

    xEventGroupSetBits(s_base_control_evt_group, BASE_CONTROL_TERM_BIT);
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Base control task stopped");
    s_base_control_task_h = NULL;
    vTaskDelete(NULL);
}

// mros_cmd_vel_cb_t signature
//! This is executed within the mros executor task
void on_cmd_vel_callback(const geometry_msgs__msg__TwistStamped *msg, void *context) {
    ESP_UNUSED(context);

    cmd_t old_cmd;
    cmd_t new_cmd = cmd_t_zero;
    sensor_msgs__msg__Imu imu_msg;
    // use local time as the timesyncronisation sometimes is of for some ms and we may want tight timeout for the internal control task
    gettimeofday(&new_cmd.timestamp, NULL);

    if (xQueuePeek(s_cmd_q, &old_cmd, 0) != pdTRUE) {
        old_cmd = cmd_t_zero;
    }

    new_cmd.correction_active = (msg->twist.linear.x > 0) ? true : false;

    if (old_cmd.correction_active == false) {
        if (mros_peek_imu_msg(&imu_msg) != ESP_OK) {
            new_cmd.correction_active = false;
            ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to peek IMU msg for initial yaw calculation.");
        }
        old_cmd.angle = math_normalize_angle(math_quaternion_to_yaw(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w));
    }

    if (new_cmd.correction_active == true) {
        // recalculate old angle only if for the old one it also was calculated, else we already use the current position as old angle
        if (old_cmd.correction_active == true) {
            wallclock_timestamp_t delta_timestamp;
            timersub(&new_cmd.timestamp, &old_cmd.timestamp, &delta_timestamp);
            // revert the prediction üto get starting angle from old command
            float starting_angle = math_normalize_angle(old_cmd.angle - old_cmd.angular_vel * (float)YAW_CORRECTION_PERIOD_MS / 1000.0f);
            old_cmd.angle = math_normalize_angle(starting_angle + old_cmd.angular_vel * ((float)delta_timestamp.tv_sec + (float)delta_timestamp.tv_usec / 1000000.0f));
        }
        new_cmd.angle = math_normalize_angle(old_cmd.angle + msg->twist.angular.z * (float)YAW_CORRECTION_PERIOD_MS / 1000.0f);
    }

    new_cmd.linear_vel = msg->twist.linear.x;
    new_cmd.angular_vel = msg->twist.angular.z;

    if (xQueueOverwrite(s_cmd_q, &new_cmd) != pdTRUE) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to overwrite cmd queue");
        return;
    }
}

esp_err_t robot_controller_init(odrive_context_t *odrive_ml_context, odrive_context_t *odrive_mr_context, servo_t *servo_context, EventGroupHandle_t error_handle, EventBits_t error_bit) {
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Initializing robot controller");

    if (odrive_ml_context == NULL || odrive_mr_context == NULL) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Invalid ODrive context");
        return ESP_ERR_INVALID_ARG;
    }

    if (odrive_ml_context->node_id == odrive_mr_context->node_id) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "ODrive Node IDs are the same");
        return ESP_ERR_INVALID_ARG;
    }

    if (odrive_ml_context->control_mode != CONTROL_MODE_VELOCITY_CONTROL || odrive_mr_context->control_mode != CONTROL_MODE_VELOCITY_CONTROL) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "ODrive is not in velocity control mode");
        return ESP_ERR_INVALID_STATE;
    }

    if (odrive_ml_context->input_mode != INPUT_MODE_PASSTHROUGH || odrive_mr_context->input_mode != INPUT_MODE_PASSTHROUGH) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "ODrive is not in passthrough input mode");
        return ESP_ERR_INVALID_STATE;
    }
    s_odrive_ml_context = odrive_ml_context;
    s_odrive_mr_context = odrive_mr_context;

    if (servo_context == NULL) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Invalid servo context");
        return ESP_ERR_INVALID_ARG;
    }
    s_servo_context = servo_context;

    s_cmd_q = xQueueCreate(1, sizeof(cmd_t));
    if (s_cmd_q == NULL) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to create cmd queue");
        return ESP_FAIL;
    }
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "cmd queue created");

    if (!error_handle) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Invalid error handle");
        return ESP_ERR_INVALID_ARG;
    }

    s_base_control_err_evt_group = error_handle;
    s_base_control_err_bit = error_bit;
    xEventGroupClearBits(s_base_control_err_evt_group, s_base_control_err_bit); // Probably not needed, but just to be sure -> so that we start with a clean slate

    s_base_control_evt_group = xEventGroupCreate();
    if (!s_base_control_evt_group) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to create event group");
        return ESP_FAIL;
    }

    xEventGroupSetBits(s_base_control_evt_group, BASE_CONTROL_TERM_BIT);
    xEventGroupClearBits(s_base_control_evt_group, BASE_CONTROL_RUN_BIT);
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Event group created");

    if (mros_register_cmd_vel_callback(on_cmd_vel_callback, NULL) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to register cmd_vel callback");
        return ESP_FAIL;
    }
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "cmd_vel callback registered");

    s_base_control_task_h = NULL;

    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Module initialized successfully");
    return ESP_OK;
}

esp_err_t robot_controller_start(void) {
    if (!s_base_control_task_h) {
        if (xTaskCreate(base_control_task, ROBOT_CONTROLLER_TASK_NAME, ROBOT_CONTROLLER_STACK_SIZE, NULL, ROBOT_CONTROLLER_PRIORITY, &s_base_control_task_h) != pdPASS) {
            ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to create base control task");
            s_base_control_task_h = NULL;
            return ESP_FAIL;
        }
    }

    xEventGroupSetBits(s_base_control_evt_group, BASE_CONTROL_RUN_BIT);

    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Module started");
    return ESP_OK;
}

esp_err_t robot_controller_stop(TickType_t wait_ticks) {
    if (!s_base_control_task_h) {
        ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Module not running");
        return ESP_OK; // Already stopped
    }

    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Stopping module...");
    xEventGroupClearBits(s_base_control_evt_group, BASE_CONTROL_RUN_BIT);

    EventBits_t bits = xEventGroupWaitBits(s_base_control_evt_group, BASE_CONTROL_TERM_BIT, pdFALSE, pdTRUE, wait_ticks);
    if (!(bits & BASE_CONTROL_TERM_BIT)) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Tasks did not stop in time");
        return ESP_FAIL;
    }
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Module stopped");
    return ESP_OK;
}

esp_err_t robot_controller_deinit(TickType_t wait_ticks) {
    if (robot_controller_stop(pdMS_TO_TICKS(wait_ticks)) != ESP_OK) {
        return ESP_FAIL;
    }

    if (s_base_control_evt_group) {
        vEventGroupDelete(s_base_control_evt_group);
        s_base_control_evt_group = NULL;
    }

    if (s_base_control_task_h) {
        vTaskDelete(s_base_control_task_h);
        s_base_control_task_h = NULL;
    }

    if (s_odrive_ml_context) {
        s_odrive_ml_context = NULL;
    }

    if (s_odrive_mr_context) {
        s_odrive_mr_context = NULL;
    }

    if (s_servo_context) {
        s_servo_context = NULL;
    }

    if (s_cmd_q) {
        vQueueDelete(s_cmd_q);
        s_cmd_q = NULL;
    }

    s_base_control_err_evt_group = NULL;

    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Module deinitialized");
    return ESP_OK;
}