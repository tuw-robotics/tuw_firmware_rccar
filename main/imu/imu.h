#ifndef IMU_H
#define IMU_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <sensor_msgs/msg/imu.h>

#define IMU_TASK_LOGGER_TAG "IMU"
#define IMU_TASK_NAME "imu_task"
#define IMU_TASK_STACK_SIZE 4096
#define IMU_TASK_PRIORITY 6

/**
 * @brief Calculates yaw from a imu message
 *
 * @param imu_msg The input IMU message
 * @return float The calculated yaw in radians
 */
float calculate_yaw_from_imu_msg(sensor_msgs__msg__Imu *imu_msg);

/**
 * @brief Initializes the IMU task
 *
 * @param error_handle Event group handle for error events
 * @param error_bit Event bit to set on error
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t imu_task_init(EventGroupHandle_t error_handle, EventBits_t error_bit);

/**
 * @brief Starts the IMU task
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t imu_task_start(void);

/**
 * @brief Deinitializes the IMU task
 *
 * @param wait_ticks Maximum time to wait for the task to deinitialize
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t imu_task_deinit(TickType_t wait_ticks);

/**
 * @brief Stops the IMU task
 *
 * @param wait_ticks Maximum time to wait for the task to stop
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t imu_task_stop(TickType_t wait_ticks);

#endif // IMU_H