#include "odom.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "kinematics/kinematics.h"
#include "mros/mros.h"
#include "mros/mros_conf.h"
#include "robot_controller/robot_controller.h"
#include "servo/servo.h"
#include "utils/timing_utils.h"
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <nav_msgs/msg/odometry.h>
#include <odrive/odrive.h>
#include <odrive/odrive_conf.h>
#include <sys/time.h>

static TaskHandle_t s_odom_publish_task_h = NULL;

static EventGroupHandle_t s_odom_publish_evt_group = NULL;
#define ODOM_PUBLISH_RUN_BIT BIT0
#define ODOM_PUBLISH_TERM_BIT BIT1

static EventGroupHandle_t s_odom_publish_err_evt_group;
static EventBits_t s_odom_publish_err_bit;

static odrive_context_t *s_odrive_ml_context;
static odrive_context_t *s_odrive_mr_context;
static servo_t *s_servo_context;

static void odom_publish_task(void *pv) {
    ESP_UNUSED(pv);
    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Odom publish task started");

    EventBits_t bits = xEventGroupWaitBits(s_odom_publish_evt_group, ODOM_PUBLISH_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY); // Wait for start signal

    nav_msgs__msg__Odometry odom_msg;
    odrive_encoder_estimates_t estimates_ml;
    odrive_encoder_estimates_t estimates_mr;
    wallclock_timestamp_t timestamp_ml;
    wallclock_timestamp_t timestamp_mr;

    float pos_x = 0.0f, pos_y = 0.0f;
    float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f; // identity orientation

    if (mros_init_odometry_msg(&odom_msg) != ESP_OK) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to initialize odometry message");
        vTaskDelete(NULL);
    }
    xEventGroupClearBits(s_odom_publish_evt_group, ODOM_PUBLISH_TERM_BIT);

    // Wait for first estimates to be available (max 10s)
    if (odrive_get_encoder_estimates(s_odrive_ml_context, &estimates_ml, &timestamp_ml, false, pdMS_TO_TICKS(S_TO_MS(10))) != ESP_OK) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to get encoder estimates for ODrive Node ID %d", s_odrive_ml_context->node_id);
        xEventGroupSetBits(s_odom_publish_err_evt_group, s_odom_publish_err_bit);
        vTaskDelete(NULL);
    }
    if (odrive_get_encoder_estimates(s_odrive_mr_context, &estimates_mr, &timestamp_mr, false, pdMS_TO_TICKS(S_TO_MS(10))) != ESP_OK) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to get encoder estimates for ODrive Node ID %d", s_odrive_mr_context->node_id);
        xEventGroupSetBits(s_odom_publish_err_evt_group, s_odom_publish_err_bit);
        vTaskDelete(NULL);
    }

    float prev_pos_ml = estimates_ml.pos_estimate;
    float prev_pos_mr = estimates_mr.pos_estimate;

    while (true) {
        bits = xEventGroupGetBits(s_odom_publish_evt_group);
        if (!(bits & ODOM_PUBLISH_RUN_BIT)) {
            break;
        }

        // Wait for encoder estimates -> block until we have them or timeout
        if (odrive_get_encoder_estimates(s_odrive_ml_context, &estimates_ml, &timestamp_ml, true, pdMS_TO_TICKS(ODRIVE_ENCODER_ESTIMATES_TIMEOUT_MS)) != ESP_OK) {
            ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to get encoder estimates for ODrive Node ID %d", s_odrive_ml_context->node_id);
            break;
        }
        if (odrive_get_encoder_estimates(s_odrive_mr_context, &estimates_mr, &timestamp_mr, true, pdMS_TO_TICKS(ODRIVE_ENCODER_ESTIMATES_TIMEOUT_MS)) != ESP_OK) {
            ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to get encoder estimates for ODrive Node ID %d", s_odrive_mr_context->node_id);
            break;
        }

        float delta_pos_ml = estimates_ml.pos_estimate - prev_pos_ml;
        float delta_pos_mr = estimates_mr.pos_estimate - prev_pos_mr;

        prev_pos_ml = estimates_ml.pos_estimate;
        prev_pos_mr = estimates_mr.pos_estimate;

        forward_kinematics_input_t fk_input = {.delta_pos_l = delta_pos_ml, .delta_pos_r = delta_pos_mr, .vel_l = estimates_ml.vel_estimate, .vel_r = estimates_mr.vel_estimate};

        forward_kinematics_output_t fk_output = {.pos_x = pos_x, .pos_y = pos_y, .qx = qx, .qy = qy, .qz = qz, .qw = qw, .vel_x = 0.0f, .omega_z = 0.0f};

        if (forward_kinematics(&fk_input, &fk_output) != ESP_OK) {
            ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to calculate forward kinematics");
            break;
        }

        pos_x = fk_output.pos_x;
        pos_y = fk_output.pos_y;
        qx = fk_output.qx;
        qy = fk_output.qy;
        qz = fk_output.qz;
        qw = fk_output.qw;

        //! We use the timestamp of the right encoder for the odometry message
        odom_msg.header.stamp.sec = timestamp_mr.tv_sec;
        odom_msg.header.stamp.nanosec = US_TO_NS(timestamp_mr.tv_usec);
        odom_msg.pose.pose.position.x = fk_output.pos_x;
        odom_msg.pose.pose.position.y = fk_output.pos_y;
        odom_msg.pose.pose.orientation.x = fk_output.qx;
        odom_msg.pose.pose.orientation.y = fk_output.qy;
        odom_msg.pose.pose.orientation.z = fk_output.qz;
        odom_msg.pose.pose.orientation.w = fk_output.qw;
        odom_msg.twist.twist.linear.x = fk_output.vel_x;
        odom_msg.twist.twist.angular.z = fk_output.omega_z;

        if (mros_update_odometry(&odom_msg) != ESP_OK) { // Mros will publish it once the internal timer is triggered
            ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Failed to update odometry");
            break;
        }
        vTaskDelay(1);
    }

    if ((bits & ODOM_PUBLISH_RUN_BIT)) {
        xEventGroupSetBits(s_odom_publish_err_evt_group, s_odom_publish_err_bit);
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Odom publish task terminated unexpectedly");
    }

    xEventGroupSetBits(s_odom_publish_evt_group, ODOM_PUBLISH_TERM_BIT);
    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Odom publish task stopped");
    s_odom_publish_task_h = NULL;
    vTaskDelete(NULL);
}

esp_err_t odom_publish_task_init(odrive_context_t *odrive_ml_context, odrive_context_t *odrive_mr_context, servo_t *servo_context, EventGroupHandle_t error_handle, EventBits_t error_bit) {
    if (!error_handle) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Invalid error handle");
        return ESP_ERR_INVALID_ARG;
    }

    if (odrive_ml_context == NULL || odrive_mr_context == NULL) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Invalid ODrive context");
        return ESP_ERR_INVALID_ARG;
    }

    if (odrive_ml_context->node_id == odrive_mr_context->node_id) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "ODrive Node IDs are the same");
        return ESP_ERR_INVALID_ARG;
    }

    if (odrive_ml_context->control_mode != CONTROL_MODE_VELOCITY_CONTROL || odrive_mr_context->control_mode != CONTROL_MODE_VELOCITY_CONTROL) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "ODrive is not in velocity control mode");
        return ESP_ERR_INVALID_STATE;
    }

    if (odrive_ml_context->input_mode != INPUT_MODE_PASSTHROUGH || odrive_mr_context->input_mode != INPUT_MODE_PASSTHROUGH) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "ODrive is not in passthrough input mode");
        return ESP_ERR_INVALID_STATE;
    }
    s_odrive_ml_context = odrive_ml_context;
    s_odrive_mr_context = odrive_mr_context;

    if (servo_context == NULL) {
        ESP_LOGE(ROBOT_CONTROLLER_LOGGER_TAG, "Invalid servo context");
        return ESP_ERR_INVALID_ARG;
    }
    s_servo_context = servo_context;

    s_odom_publish_err_evt_group = error_handle;
    s_odom_publish_err_bit = error_bit;
    xEventGroupClearBits(s_odom_publish_err_evt_group, s_odom_publish_err_bit);

    s_odom_publish_evt_group = xEventGroupCreate();
    if (!s_odom_publish_evt_group) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to create Odom event group");
        return ESP_FAIL;
    }

    xEventGroupSetBits(s_odom_publish_evt_group, ODOM_PUBLISH_TERM_BIT);
    xEventGroupClearBits(s_odom_publish_evt_group, ODOM_PUBLISH_RUN_BIT);
    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Event group created");

    s_odom_publish_task_h = NULL;
    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Module initialized successfully");
    return ESP_OK;
}

esp_err_t odom_publish_task_start(void) {
    if (!s_odom_publish_task_h) {
        if (xTaskCreate(odom_publish_task, ODOM_PUBLISH_TASK_NAME, ODOM_PUBLISH_STACK_SIZE, NULL, ODOM_PUBLISH_PRIORITY, &s_odom_publish_task_h) != pdPASS) {
            ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Failed to create Odom publish task");
            s_odom_publish_task_h = NULL;
            return ESP_FAIL;
        }
    }
    xEventGroupSetBits(s_odom_publish_evt_group, ODOM_PUBLISH_RUN_BIT);
    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Module started");
    return ESP_OK;
}

esp_err_t odom_publish_task_stop(TickType_t wait_ticks) {
    if (!s_odom_publish_task_h) {
        ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Module not running");
        return ESP_OK; // Already stopped
    }

    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Stopping module...");
    xEventGroupClearBits(s_odom_publish_evt_group, ODOM_PUBLISH_RUN_BIT);

    EventBits_t bits = xEventGroupWaitBits(s_odom_publish_evt_group, ODOM_PUBLISH_TERM_BIT, pdFALSE, pdTRUE, wait_ticks);
    if (!(bits & ODOM_PUBLISH_TERM_BIT)) {
        ESP_LOGE(ODOM_PUBLISH_LOGGER_TAG, "Tasks did not stop in time");
        return ESP_FAIL;
    }
    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Module stopped");
    return ESP_OK;
}

esp_err_t odom_publish_task_deinit(TickType_t wait_ticks) {
    if (odom_publish_task_stop(wait_ticks) != ESP_OK) {
        return ESP_FAIL;
    }

    if (s_odom_publish_evt_group) {
        vEventGroupDelete(s_odom_publish_evt_group);
        s_odom_publish_evt_group = NULL;
    }

    if (s_odom_publish_task_h) {
        vTaskDelete(s_odom_publish_task_h);
        s_odom_publish_task_h = NULL;
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

    s_odom_publish_err_evt_group = NULL;

    ESP_LOGI(ODOM_PUBLISH_LOGGER_TAG, "Module deinitialized");
    return ESP_OK;
}