#include "robot_controller.h"

#include "kinematics/kinematics.h"
#include "mros/mros.h"
#include "odrive/odrive.h"
#include "odrive/odrive_types.h"
#include "servo/servo.h"
#include "utils/timing_utils.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <sys/time.h>

static odrive_context_t *s_odrive_ml_context;
static odrive_context_t *s_odrive_mr_context;
static servo_t *s_servo_context;

static TaskHandle_t s_base_control_task_h;

static EventGroupHandle_t s_base_control_evt_group;
#define BASE_CONTROL_RUN_BIT BIT0
#define BASE_CONTROL_TERM_BIT BIT1

static EventGroupHandle_t s_base_control_err_evt_group; // This is for external systems to be able to react to an error
static EventBits_t s_base_control_err_bit;

static void base_control_task(void *pv) {
    ESP_UNUSED(pv);
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Base control task started");

    EventBits_t bits = xEventGroupWaitBits(s_base_control_evt_group, BASE_CONTROL_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY); // Wait for start signal

    while (true) {
        bits = xEventGroupGetBits(s_base_control_evt_group);
        if (!(bits & BASE_CONTROL_RUN_BIT)) {
            break; // Instead of terminating every time the run bit is unset, we could just set it to a waiting state -> can be restarted again
        }
        vTaskDelay(1);
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

    float torque_ff_ml = 0.0f;
    float torque_ff_mr = 0.0f;

    inverse_kinematics_input_t ik_input = {.vel_x = msg->twist.linear.x, .omega_z = msg->twist.angular.z};

    inverse_kinematics_output_t ik_output;

    if (inverse_kinematics(&ik_input, &ik_output) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to calculate inverse kinematics");
        return;
    }

    // ESP_LOGD(ROBOT_CONTROLLER_LOGGER_TAG, "Calculated velocities: ml: %f rad/s = %f rev/s, mr: %f rad/s = %f rev/s", ik_output.vel_l, RAD_TO_REV(ik_output.vel_l), ik_output.vel_r, RAD_TO_REV(ik_output.vel_r));

    if (odrive_set_velocity(s_odrive_ml_context, RAD_TO_REV(ik_output.vel_l), torque_ff_ml) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set velocity for ODrive Node ID %d", s_odrive_ml_context->node_id);
        return;
    }
    if (odrive_set_velocity(s_odrive_mr_context, RAD_TO_REV(ik_output.vel_r), torque_ff_mr) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set velocity for ODrive Node ID %d", s_odrive_mr_context->node_id);
        return;
    }

#ifdef JITTER_DEBUG
    // Calculate the jitter
    wallclock_timestamp_t timestamp_local;
    wallclock_timestamp_t timestamp_header;
    gettimeofday(&timestamp_local, NULL);

    timestamp_header.tv_sec = msg->header.stamp.sec;
    timestamp_header.tv_usec = msg->header.stamp.nanosec / 1000; // Convert to microseconds

    //! The timestamp from sending the cmd_vel until the calculated setpoint are sent to the ODrive
    //! Accuracy is dependent on synchronization of the system clock and the timestamp of the cmd_vel message
    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Header: %lld.%06ld, Local: %lld.%06ld", timestamp_header.tv_sec, timestamp_header.tv_usec, timestamp_local.tv_sec, timestamp_local.tv_usec);

#endif // JITTER_DEBUG

    if (servo_set_angle(s_servo_context, RAD_TO_DEG(ik_output.steer)) != ESP_OK) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to set steering angle");
        return;
    }

    // ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Set velocities: ml: %f rad/s = %f rev/s, mr: %f rad/s = %f rev/s, steer angle: %f deg", ik_output.vel_l, RAD_TO_REV(ik_output.vel_l), ik_output.vel_r,
    // RAD_TO_REV(ik_output.vel_r), RAD_TO_DEG(ik_output.steer));
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

    s_base_control_err_evt_group = NULL;

    ESP_LOGI(ROBOT_CONTROLLER_LOGGER_TAG, "Module deinitialized");
    return ESP_OK;
}