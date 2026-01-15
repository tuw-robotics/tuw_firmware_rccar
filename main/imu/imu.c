#include "imu.h"

#include "bno055/bno_wrapper.h"
#include "mros/mros.h"
#include "utils/math_utils.h"
#include "utils/timing_utils.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <sdkconfig.h>
#include <sensor_msgs/msg/imu.h>

static TaskHandle_t s_imu_task_h;

static EventGroupHandle_t s_imu_evt_group = NULL;
#define IMU_RUN_BIT BIT0
#define IMU_TERM_BIT BIT1

static EventGroupHandle_t s_imu_err_evt_group = NULL;
static EventBits_t s_imu_err_bit;

float calculate_yaw_from_imu_msg(sensor_msgs__msg__Imu *imu_msg) {
    float siny_cosp = 2.0f * (imu_msg->orientation.w * imu_msg->orientation.z + imu_msg->orientation.x * imu_msg->orientation.y);
    float cosy_cosp = 1.0f - 2.0f * (imu_msg->orientation.y * imu_msg->orientation.y + imu_msg->orientation.z * imu_msg->orientation.z);
    return math_normalize_angle(atan2f(siny_cosp, cosy_cosp));
}

static esp_err_t imu_data(sensor_msgs__msg__Imu *imu_msg) {
    if (!imu_msg) {
        return ESP_ERR_INVALID_ARG;
    }

    imu_msg->header.stamp = time_now();

    bno_quaternion_t quat = bno_get_quaternion();
    bno_vector_t ang_vel = bno_get_angular_velocity();
    bno_vector_t lin_acc = bno_get_linear_acceleration();

    imu_msg->orientation.x = quat.x;
    imu_msg->orientation.y = quat.y;
    imu_msg->orientation.z = quat.z;
    imu_msg->orientation.w = quat.w;
    imu_msg->angular_velocity.x = ang_vel.x;
    imu_msg->angular_velocity.y = ang_vel.y;
    imu_msg->angular_velocity.z = ang_vel.z;
    imu_msg->linear_acceleration.x = lin_acc.x;
    imu_msg->linear_acceleration.y = lin_acc.y;
    imu_msg->linear_acceleration.z = lin_acc.z;

    return ESP_OK;
}

static void imu_task(void *pv) {
    ESP_UNUSED(pv);
    ESP_LOGI(IMU_TASK_LOGGER_TAG, "IMU task started");

    EventBits_t bits = xEventGroupWaitBits(s_imu_evt_group, IMU_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY); // Wait for start signal

    sensor_msgs__msg__Imu imu_msg;

    if (mros_init_imu_msg(&imu_msg) != ESP_OK) {
        ESP_LOGE(IMU_TASK_LOGGER_TAG, "Failed to initialize IMU message");
        vTaskDelete(NULL);
    }
    xEventGroupClearBits(s_imu_evt_group, IMU_TERM_BIT);

    while (true) {
        bits = xEventGroupGetBits(s_imu_evt_group);
        if (!(bits & IMU_RUN_BIT)) {
            break;
        }

        if (imu_data(&imu_msg) != ESP_OK) {
            ESP_LOGE(IMU_TASK_LOGGER_TAG, "Failed to get IMU data");
            break;
        }

        if (mros_update_imu(&imu_msg) != ESP_OK) {
            ESP_LOGE(IMU_TASK_LOGGER_TAG, "Failed to update IMU data");
            break;
        }
        vTaskDelay(1);
    }

    if ((bits & IMU_RUN_BIT)) {
        xEventGroupSetBits(s_imu_err_evt_group, s_imu_err_bit);
        ESP_LOGE(IMU_TASK_LOGGER_TAG, "IMU task terminated unexpectedly");
    }

    xEventGroupSetBits(s_imu_evt_group, IMU_TERM_BIT);
    ESP_LOGI(IMU_TASK_LOGGER_TAG, "IMU task stopped");
    s_imu_task_h = NULL;
    vTaskDelete(NULL);
}

esp_err_t imu_task_init(EventGroupHandle_t error_handle, EventBits_t error_bit) {
    if (!error_handle) {
        ESP_LOGE(IMU_TASK_LOGGER_TAG, "Invalid error handle");
        return ESP_ERR_INVALID_ARG;
    }

    s_imu_err_evt_group = error_handle;
    s_imu_err_bit = error_bit;
    xEventGroupClearBits(s_imu_err_evt_group, s_imu_err_bit);

    s_imu_evt_group = xEventGroupCreate();
    if (!s_imu_evt_group) {
        ESP_LOGE(IMU_TASK_LOGGER_TAG, "Failed to create IMU event group");
        return ESP_FAIL;
    }

    xEventGroupSetBits(s_imu_evt_group, IMU_TERM_BIT);
    xEventGroupClearBits(s_imu_evt_group, IMU_RUN_BIT);
    ESP_LOGI(IMU_TASK_LOGGER_TAG, "Event group created");

    s_imu_task_h = NULL;
    ESP_LOGI(IMU_TASK_LOGGER_TAG, "Module initialized successfully");
    return ESP_OK;
}

esp_err_t imu_task_start(void) {
    if (!s_imu_task_h) {
        if (xTaskCreate(imu_task, IMU_TASK_NAME, IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, &s_imu_task_h) != pdPASS) {
            ESP_LOGE(IMU_TASK_LOGGER_TAG, "Failed to create IMU task");
            s_imu_task_h = NULL;
            return ESP_FAIL;
        }
    }

    xEventGroupSetBits(s_imu_evt_group, IMU_RUN_BIT);

    ESP_LOGI(IMU_TASK_LOGGER_TAG, "Module started");
    return ESP_OK;
}

esp_err_t imu_task_stop(TickType_t wait_ticks) {
    if (!s_imu_task_h) {
        ESP_LOGI(IMU_TASK_LOGGER_TAG, "Module not running");
        return ESP_OK; // Already stopped
    }

    ESP_LOGI(IMU_TASK_LOGGER_TAG, "Stopping module...");
    xEventGroupClearBits(s_imu_evt_group, IMU_RUN_BIT);

    EventBits_t bits = xEventGroupWaitBits(s_imu_evt_group, IMU_TERM_BIT, pdFALSE, pdTRUE, wait_ticks);
    if (!(bits & IMU_TERM_BIT)) {
        ESP_LOGE(IMU_TASK_LOGGER_TAG, "Tasks did not stop in time");
        return ESP_FAIL;
    }
    ESP_LOGI(IMU_TASK_LOGGER_TAG, "Module stopped");
    return ESP_OK;
}

esp_err_t imu_task_deinit(TickType_t wait_ticks) {
    if (imu_task_stop(wait_ticks) != ESP_OK) {
        return ESP_FAIL;
    }

    if (s_imu_evt_group) {
        vEventGroupDelete(s_imu_evt_group);
        s_imu_evt_group = NULL;
    }

    if (s_imu_task_h) {
        vTaskDelete(s_imu_task_h);
        s_imu_task_h = NULL;
    }

    s_imu_err_evt_group = NULL;

    ESP_LOGI(IMU_TASK_LOGGER_TAG, "Module deinitialized successfully");
    return ESP_OK;
}