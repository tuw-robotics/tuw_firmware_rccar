#ifndef ODOM_H
#define ODOM_H

#include "odrive/odrive.h"
#include "odrive/odrive_enums.h"
#include "odrive/odrive_types.h"
#include "servo/servo.h"

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#define ODOM_TASK_LOGGER_TAG "ODOM"
#define ODOM_TASK_NAME "odom_task"
#define ODOM_TASK_STACK_SIZE 4096
#define ODOM_TASK_PRIORITY 6

/**
 * @brief Initializes the odom task
 *
 * @param odrive_ml_context Context for the left ODrive motor
 * @param odrive_mr_context Context for the right ODrive motor
 * @param servo_context Context for the servo motor
 * @param error_handle Event group handle for error events
 * @param error_bit Event bit to set on error
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odom_task_init(odrive_context_t *odrive_ml_context, odrive_context_t *odrive_mr_context, servo_t *servo_context, EventGroupHandle_t error_handle, EventBits_t error_bit);

/**
 * @brief Starts the odom task
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odom_task_start(void);

/**
 * @brief Deinitializes the odom task
 *
 * @param wait_ticks Maximum time to wait for the task to deinitialize
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odom_task_deinit(TickType_t wait_ticks);

/**
 * @brief Stops the odom task
 *
 * @param wait_ticks Maximum time to wait for the task to stop
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odom_task_stop(TickType_t wait_ticks);

#endif // ODOM_H