#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "odrive/odrive.h"
#include "odrive/odrive_enums.h"
#include "odrive/odrive_types.h"
#include "servo/servo.h"

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#define ROBOT_CONTROLLER_LOGGER_TAG "robot_controller"
#define ROBOT_CONTROLLER_TASK_NAME "robot_controller_task"
#define ROBOT_CONTROLLER_STACK_SIZE 4096
#define ROBOT_CONTROLLER_PRIORITY 5

/**
 * @brief Initializes the robot controller module
 *
 * @param odrive_ml_context Context for the left ODrive motor
 * @param odrive_mr_context Context for the right ODrive motor
 * @param servo_context Context for the servo motor
 * @param error_handle Event group handle for error events
 * @param error_bit Event bit to set on error
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t robot_controller_init(odrive_context_t *odrive_ml_context, odrive_context_t *odrive_mr_context, servo_t *servo_context, EventGroupHandle_t error_handle, EventBits_t error_bit);

/**
 * @brief Starts the robot controller module
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t robot_controller_start(void);

/**
 * @brief Stops the robot controller module
 *
 * @param wait_ticks Maximum time to wait for the module to stop
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t robot_controller_stop(TickType_t wait_ticks);

/**
 * @brief Deinitializes the robot controller module
 *
 * @param wait_ticks Maximum time to wait for the module to deinitialize
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t robot_controller_deinit(TickType_t wait_ticks);

#endif // ROBOT_CONTROLLER_H
