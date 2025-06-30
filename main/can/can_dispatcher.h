#ifndef CAN_DISPATCHER_H
#define CAN_DISPATCHER_H

#include "can_conf.h"
#include "can_driver.h"
#include "can_interface.h"
#include <driver/twai.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t msg_id;
    can_message_callback_t cb;
    void *ctx;
} disp_entry_t;

/**
 * @brief Initializes the CAN dispatcher
 *
 * @param error_handle Event group handle for error events
 * @param error_bit Event bit to set on error
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_dispatcher_init(EventGroupHandle_t error_handle, EventBits_t error_bit);

/**
 * @brief Registers a callback for a specific CAN message ID
 *
 * @param msg_id The ID of the CAN message to register the callback for
 * @param cb The callback function to be called when the message is received
 * @param ctx User-defined context to be passed to the callback function
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 * @note Multiple callbacks can be registered for the same message ID. But a maximum
 * of MAX_CBS callbacks can be registered in total
 */
esp_err_t can_dispatcher_register(uint32_t msg_id, can_message_callback_t cb, void *ctx);

/**
 * @brief Unregisters a callback for a specific CAN message ID
 *
 * @param msg_id The ID of the CAN message to unregister the callback for
 * @param cb The callback function to be unregistered
 * @param ctx User-defined context to be passed to the callback function
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_dispatcher_unregister(uint32_t msg_id, can_message_callback_t cb, void *ctx);

/**
 * @brief Starts the CAN dispatcher task
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 * @note If the can driver is not running, it will be started automatically
 */
esp_err_t can_dispatcher_start(void);

/**
 * @brief Stops the CAN dispatcher task
 *
 * @param wait_ticks The maximum time to wait for the task to terminate
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_dispatcher_stop(TickType_t wait_ticks);

/**
 * @brief Deinitializes the CAN dispatcher
 *
 * @param wait_ticks The maximum time to wait for the task to terminate
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 * @note This will also stop the dispatcher task if it is running
 */
esp_err_t can_dispatcher_deinit(TickType_t wait_ticks);

#endif // CAN_DISPATCHER_H