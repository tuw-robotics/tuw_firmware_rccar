
#ifndef ODRIVE_H
#define ODRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "odrive/odrive_can.h"
#include "odrive/odrive_conf.h"
#include "odrive/odrive_enums.h"
#include "odrive/odrive_types.h"

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t node_id;
    QueueHandle_t heartbeat_queue;
    QueueHandle_t encoder_estimates_queue;
    ControlMode control_mode;
    InputMode input_mode;
    int8_t direction;
} odrive_context_t;

/**
 * @brief Initializes an ODrive context
 *
 * @param context Pointer to the ODrive context to initialize
 * @param node_id Configured ODrive node ID (0-31)
 * @param control_mode Control mode to be set for the ODrive
 * @param input_mode Input mode to be set for the ODrive
 * @param direction Direction of the motor (1 for forward, -1 for reverse)
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_context_init(odrive_context_t *context, uint8_t node_id, ControlMode control_mode, InputMode input_mode, int8_t direction);

/**
 * @brief Cleans up an ODrive context
 *
 * @param context Pointer to the ODrive context to clean up
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_context_cleanup(odrive_context_t *context);

/**
 * @brief Prepares the ODrive for initial state as configuration
 *
 * @param context Pointer to the ODrive context to prepare
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_prepare_initial_state(odrive_context_t *context);

/**
 * @brief Starts the ODrive startup sequence
 *
 * This function performs the following steps:
 * 1. Discovers the ODrive node
 * 2. Checks for initial heartbeat reception
 * 3. Prepares the initial state (waits for IDLE if necessary)
 * 4. Sets the control mode
 * 5. Sets the axis state to CLOSED_LOOP_CONTROL
 *
 * @param context Pointer to the ODrive context to start up
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_startup(odrive_context_t *context);

/**
 * @brief Discovers the ODrive node
 *
 * @param context Pointer to the ODrive context to discover
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_discover(odrive_context_t *context);

/**
 * @brief Gets the current axis state of the ODrive
 *
 * @param context Pointer to the ODrive context
 * @param axis_state Pointer to store the current axis state
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_get_axis_state(odrive_context_t *context, AxisState *axis_state);

/**
 * @brief Checks if the ODrive is in the expected axis state
 *
 * @param context Pointer to the ODrive context
 * @param expected_axis_state The expected axis state to check against
 * @return esp_err_t ESP_OK if in expected state, ESP_FAIL otherwise
 */
esp_err_t odrive_check_state(odrive_context_t *context, AxisState expected_axis_state);

/**
 * @brief Waits for the ODrive to reach a specific axis state
 *
 * @param context Pointer to the ODrive context
 * @param target_axis_state The target axis state to wait for
 * @param timeout Maximum time to wait for the target state
 * @return esp_err_t ESP_OK if reached, ESP_ERR_TIMEOUT if timed out, ESP_FAIL on error
 */
esp_err_t odrive_wait_state(odrive_context_t *context, AxisState target_axis_state, TickType_t timeout);

/**
 * @brief Sets the control mode and input mode for the ODrive
 *
 * @param context Pointer to the ODrive context
 * @param control_mode The control mode to set
 * @param input_mode The input mode to set
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_set_control_mode(odrive_context_t *context, ControlMode control_mode, InputMode input_mode);

/**
 * @brief Sets the axis state of the ODrive
 *
 * @param context Pointer to the ODrive context
 * @param axis_state The axis state to set
 * @param confirm Whether to confirm the state change
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_set_axis_state(odrive_context_t *context, AxisState axis_state, bool confirm);

/**
 * @brief Gets the encoder estimates from the ODrive
 *
 * @param context Pointer to the ODrive context
 * @param estimates Pointer to store the encoder estimates
 * @param timestamp Pointer to store the timestamp of the estimates
 * @param consume Whether to consume the queue item (true) or just peek it (false)
 * @param timeout Maximum time to wait for the estimates
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_get_encoder_estimates(odrive_context_t *context, odrive_encoder_estimates_t *estimates, wallclock_timestamp_t *timestamp, bool consume, TickType_t timeout);

/**
 * @brief Sets the velocity of the ODrive
 *
 * @param context Pointer to the ODrive context
 * @param velocity The desired velocity in revolutions per second
 * @param torque_feed_forward The feed-forward torque in Newton-meters
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_set_velocity(odrive_context_t *context, float velocity, float torque_feed_forward);

/**
 * @brief Sets the position of the ODrive
 *
 * @param context Pointer to the ODrive context
 * @param position The desired position in revolutions
 * @param velocity_feed_forward The feed-forward velocity in revolutions per second
 * @param torque_feed_forward The feed-forward torque in Newton-meters
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_set_position(odrive_context_t *context, float position, int16_t velocity_feed_forward, int16_t torque_feed_forward);

/**
 * @brief Sets the torque of the ODrive
 *
 * @param context Pointer to the ODrive context
 * @param torque The desired torque in Newton-meters
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t odrive_set_torque(odrive_context_t *context, float torque);

#ifdef __cplusplus
}
#endif

#endif // ODRIVE_H
