#ifndef MROS_INTERFACE_H
#define MROS_INTERFACE_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <sdkconfig.h>
#include <stdbool.h>

#include "utils/timing_utils.h"

#include "mros_conf.h"

typedef void (*mros_cmd_vel_cb_t)(const geometry_msgs__msg__TwistStamped *msg, void *context);

/**
 * @brief Initializes the MROS module
 *
 * @param error_handle Event group handle for error events
 * @param error_bit Event bit to set on error
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mros_module_init(EventGroupHandle_t error_handle, EventBits_t error_bit);

/**
 * @brief Starts the MROS module
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mros_module_start(void);

/**
 * @brief Stops the MROS module
 *
 * @param wait_ticks Maximum time to wait for the module to stop
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mros_module_stop(TickType_t wait_ticks);

/**
 * @brief Deinitializes the MROS module
 *
 * @param wait_ticks Maximum time to wait for the module to deinitialize
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mros_module_deinit(TickType_t wait_ticks);

/**
 * @brief Registers a callback for the cmd_vel topic
 *
 * @param callback The callback function to be called when a cmd_vel message is received
 * @param context User-defined context to be passed to the callback function
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t mros_register_cmd_vel_callback(mros_cmd_vel_cb_t callback, void *context);

/**
 * @brief Initializes the odometry message
 *
 * @param odom_msg Pointer to the odometry message to be initialized
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if odom_msg is NULL
 */
esp_err_t mros_init_odometry_msg(nav_msgs__msg__Odometry *odom_msg);

/**
 * @brief Updates the odometry message
 *
 * @param odom_msg Pointer to the odometry message to be updated
 * @return esp_err_t ESP_OK on success, ESP_FAIL if the queue overwrite fails
 */
esp_err_t mros_update_odometry(nav_msgs__msg__Odometry *odom_msg);

/**
 * @brief Checks if the MROS agent is connected
 *
 * @return true if the agent is connected, false otherwise
 */
bool mros_is_agent_connected(void);

/**
 * @brief Checks if the MROS time is synchronized
 *
 * @return true if the time is synchronized, false otherwise
 */
bool mros_is_time_synced(void);

#endif // MROS_INTERFACE_H