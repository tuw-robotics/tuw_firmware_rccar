#include "can_interface.h"
#include "can_dispatcher.h"
#include "can_driver.h"
#include "can_rtr.h"
#include "utils/timing_utils.h"
#include <esp_log.h>
#include <string.h>

static volatile bool s_can_running = false; // TODO: Replace with more detailed state management and bitmasks (or event groups)
static volatile bool s_can_initialized = false;

esp_err_t can_module_init(EventGroupHandle_t error_handle, EventBits_t error_bit) {
    if (s_can_initialized) {
        ESP_LOGW(CAN_LOGGER_TAG, "CAN module already initialized");
        return ESP_OK;
    }
    if (can_driver_init_default() != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to initialize CAN driver");
        return ESP_FAIL;
    }

    if (can_rtr_init() != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to initialize CAN RTR");
        if (can_driver_deinit() != ESP_OK) {
            ESP_LOGW(CAN_LOGGER_TAG, "Failed to deinitialize CAN driver after RTR init failure");
        }
        return ESP_FAIL;
    }

    if (can_dispatcher_init(error_handle, error_bit) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to initialize CAN dispatcher");
        if (can_rtr_deinit() != ESP_OK) {
            ESP_LOGW(CAN_LOGGER_TAG, "Failed to deinitialize CAN RTR after dispatcher init failure");
        }
        if (can_driver_deinit() != ESP_OK) {
            ESP_LOGW(CAN_LOGGER_TAG, "Failed to deinitialize CAN driver after dispatcher init failure");
        }
        return ESP_FAIL;
    }

    s_can_initialized = true;

    ESP_LOGI(CAN_LOGGER_TAG, "CAN module initialized successfully");
    return ESP_OK;
}

esp_err_t can_module_start(void) {
    if (s_can_running) {
        ESP_LOGW(CAN_LOGGER_TAG, "CAN module already running");
        return ESP_OK;
    }
    if (!s_can_initialized) {
        ESP_LOGE(CAN_LOGGER_TAG, "CAN module not initialized");
        return ESP_FAIL;
    }
    if (can_driver_start() != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to start CAN driver");
        return ESP_FAIL;
    }
    if (can_dispatcher_start() != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to start CAN dispatcher task");
        return ESP_FAIL;
    }
    s_can_running = true;
    ESP_LOGI(CAN_LOGGER_TAG, "CAN module started successfully");
    return ESP_OK;
}

esp_err_t can_module_stop(TickType_t wait_ticks) {
    if (!s_can_running) {
        ESP_LOGW(CAN_LOGGER_TAG, "CAN module already stopped");
        return ESP_OK;
    }
    if (!s_can_initialized) {
        ESP_LOGE(CAN_LOGGER_TAG, "CAN module not initialized");
        return ESP_FAIL;
    }
    if (can_dispatcher_stop(wait_ticks) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to stop CAN dispatcher task within timeout");
        return ESP_FAIL;
    }
    if (can_driver_stop() != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to stop CAN driver");
        return ESP_FAIL;
    }
    s_can_running = false;
    return ESP_OK;
}

esp_err_t can_module_deinit(TickType_t wait_ticks) {
    if (!s_can_initialized) {
        ESP_LOGW(CAN_LOGGER_TAG, "CAN module already deinitialized");
        return ESP_OK;
    }
    // Not sure if we should return fail if any of these fail
    if (can_dispatcher_deinit(wait_ticks) != ESP_OK) {
        ESP_LOGW(CAN_LOGGER_TAG, "Failed to stop CAN dispatcher task within timeout during deinit");
    }
    if (can_rtr_deinit() != ESP_OK) {
        ESP_LOGW(CAN_LOGGER_TAG, "Failed to deinitialize CAN RTR during deinit");
    }
    if (can_driver_deinit() != ESP_OK) {
        ESP_LOGW(CAN_LOGGER_TAG, "Failed to deinitialize CAN driver during deinit");
    }

    s_can_initialized = false;
    s_can_running = false;
    ESP_LOGI(CAN_LOGGER_TAG, "CAN module deinitialized");
    return ESP_OK;
}

esp_err_t can_send_data(uint32_t msg_id, const void *data, size_t data_length, TickType_t transmit_timeout) {
    if (!s_can_running) {
        ESP_LOGE(CAN_LOGGER_TAG, "CAN module not running");
        return ESP_FAIL;
    }
    if (data == NULL && data_length > 0) {
        ESP_LOGE(CAN_LOGGER_TAG, "Data pointer is NULL for non-zero length");
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t tx_msg = {
        // TODO: This is the only twai dependency in this file, we should probably define our own struct in the driver
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .reserved = 0,
        .identifier = msg_id,
        .data_length_code = data_length,
    };

    if (data_length > 0) {
        memcpy(tx_msg.data, data, data_length);
    }

    if (can_driver_transmit(&tx_msg, transmit_timeout) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to transmit CAN message with ID 0x%03lX", msg_id);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t can_send_rtr_request(uint32_t msg_id, size_t data_length, TickType_t transmit_timeout, monotonic_timestamp_t rtr_timeout) {
    if (!s_can_running) {
        ESP_LOGE(CAN_LOGGER_TAG, "CAN module not running");
        return ESP_FAIL;
    }

    if (can_expect_rtr_request(msg_id, rtr_timeout) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to register RTR expectation for ID 0x%03lX", msg_id);
        return ESP_FAIL;
    }

    twai_message_t rtr_msg = {
        .extd = 0,
        .rtr = 1,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .reserved = 0,
        .identifier = msg_id,
        .data_length_code = data_length,
    };

    if (can_driver_transmit(&rtr_msg, transmit_timeout) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to transmit RTR frame for ID 0x%03lX", msg_id);
        if (can_rtr_remove_request() != ESP_OK) {
            ESP_LOGW(CAN_LOGGER_TAG, "Failed to remove RTR request after transmission failure");
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t can_receive_rtr_data(uint32_t msg_id, void *response_data, size_t response_size, TickType_t receive_timeout) {
    if (!s_can_running) {
        ESP_LOGE(CAN_LOGGER_TAG, "CAN module not running");
        return ESP_FAIL;
    }

    if (response_data == NULL && response_size > 0) {
        ESP_LOGE(CAN_LOGGER_TAG, "Response data pointer is NULL for non-zero length");
        return ESP_ERR_INVALID_ARG;
    }

    can_rtr_resp_t resp;
    if (can_rtr_get_response(&resp, receive_timeout) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to receive RTR response for ID 0x%03lX", msg_id);
        return ESP_FAIL;
    }

    if (resp.msg.identifier != msg_id) {
        ESP_LOGE(CAN_LOGGER_TAG, "Received RTR response with unexpected ID 0x%03lX (expected 0x%03lX)", resp.msg.identifier, msg_id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(response_data, resp.msg.data, response_size);

    if (resp.timed_out) {
        ESP_LOGW(CAN_LOGGER_TAG, "RTR response timed out for ID 0x%03lX", msg_id);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool can_is_running(void) { return s_can_running; }
bool can_is_initialized(void) { return s_can_initialized; }

esp_err_t can_register_message_handler(uint32_t msg_id, can_message_callback_t cb, void *ctx) { return can_dispatcher_register(msg_id, cb, ctx); }

esp_err_t can_unregister_message_handler(uint32_t msg_id, can_message_callback_t cb, void *ctx) { return can_dispatcher_unregister(msg_id, cb, ctx); }