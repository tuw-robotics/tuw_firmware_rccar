#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <driver/twai.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stddef.h>
#include <stdint.h>

#include "utils/timing_utils.h"

typedef void (*can_message_callback_t)(uint32_t msg_id, const uint8_t *data, size_t dlc, wallclock_timestamp_t wall_time, monotonic_timestamp_t monotonic_time, void *ctx);

/**
 * @brief Initialize the CAN module
 *
 * @param error_handle Event group handle for error events
 * @param error_bit Event bit to set on error
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_module_init(EventGroupHandle_t error_handle, EventBits_t error_bit); // TODO: Also provide interface for "not default"

/**
 * @brief Check if the CAN module is running
 *
 * @return true if the CAN module is running, false otherwise
 */
bool can_is_running(void);

/**
 * @brief Check if the CAN module is initialized
 *
 * @return true if the CAN module is initialized, false otherwise
 */
bool can_is_initialized(void);

/**
 * @brief Start the CAN module
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_module_start(void);

/**
 * @brief Stop the CAN module
 *
 * @param wait_ticks Maximum time to wait for the module to stop
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_module_stop(TickType_t wait_ticks);

/**
 * @brief Deinitialize the CAN module
 *
 * @param wait_ticks Maximum time to wait for the module to deinitialize
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_module_deinit(TickType_t wait_ticks);

/**
 * @brief Send a CAN message
 *
 * @param msg_id The ID of the CAN message to be sent
 * @param data Pointer to the data to be sent
 * @param data_length Length of the data to be sent
 * @param block_time Maximum time to wait for the message to be sent
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_send_data(uint32_t msg_id, const void *data, size_t data_length, TickType_t block_time);

/**
 * @brief Send a CAN RTR request
 *
 * @param msg_id The ID of the CAN message to be requested
 * @param data_length Length of the data to be requested
 * @param transmit_timeout Maximum time to wait for the message to be transmitted
 * @param rtr_timeout Timeout for the RTR request in microseconds
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_send_rtr_request(uint32_t msg_id, size_t data_length, TickType_t transmit_timeout, monotonic_timestamp_t rtr_timeout);

/**
 * @brief Receive a CAN RTR response
 *
 * @param msg_id The ID of the CAN message to be received
 * @param response_data Pointer to the buffer where the response data will be stored
 * @param response_size Size of the response data buffer
 * @param receive_timeout Maximum time to wait for the response
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if the response timed out, ESP_ERR_INVALID_RESPONSE if the response ID does not match the expected ID, ESP_FAIL on failure
 * @note If the response is timed out, it is still available in the response_data buffer
 */
esp_err_t can_receive_rtr_data(uint32_t msg_id, void *response_data, size_t response_size, TickType_t receive_timeout);

/**
 * @brief Register a callback for a specific CAN message ID
 *
 * @param msg_id The ID of the CAN message to register the callback for
 * @param cb The callback function to be called when the message is received
 * @param ctx User-defined context to be passed to the callback function
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_register_message_handler(uint32_t msg_id, can_message_callback_t cb, void *ctx);

/**
 * @brief Unregister a callback for a specific CAN message ID
 *
 * @param msg_id The ID of the CAN message to unregister the callback for
 * @param cb The callback function to be unregistered
 * @param ctx User-defined context to be passed to the callback function
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_unregister_message_handler(uint32_t msg_id, can_message_callback_t cb, void *ctx);

#endif // CAN_INTERFACE_H