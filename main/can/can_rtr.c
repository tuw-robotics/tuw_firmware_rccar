#include "can_rtr.h"
#include "can_conf.h"
#include "can_driver.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/queue.h>
#include <string.h>
#include <sys/time.h>

static QueueHandle_t s_rtr_req_q = NULL;
static QueueHandle_t s_rtr_resp_q = NULL;

esp_err_t can_rtr_init() {
    //! We currently only allow one request at a time, so we use a queue of size 1
    s_rtr_req_q = xQueueCreate(1, sizeof(can_rtr_req_t));
    s_rtr_resp_q = xQueueCreate(1, sizeof(can_rtr_resp_t));

    if (s_rtr_req_q == NULL || s_rtr_resp_q == NULL) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to create RTR queues");
        if (s_rtr_req_q)
            vQueueDelete(s_rtr_req_q);
        if (s_rtr_resp_q)
            vQueueDelete(s_rtr_resp_q);
        s_rtr_req_q = NULL;
        s_rtr_resp_q = NULL;
        return ESP_FAIL;
    }
    xQueueReset(s_rtr_req_q);
    xQueueReset(s_rtr_resp_q);
    ESP_LOGI(CAN_LOGGER_TAG, "RTR queues created");
    return ESP_OK;
}

esp_err_t can_rtr_deinit(void) {
    if (s_rtr_req_q) {
        vQueueDelete(s_rtr_req_q);
        s_rtr_req_q = NULL;
        ESP_LOGI(CAN_LOGGER_TAG, "RTR request queue deleted");
    }
    if (s_rtr_resp_q) {
        vQueueDelete(s_rtr_resp_q);
        s_rtr_resp_q = NULL;
        ESP_LOGI(CAN_LOGGER_TAG, "RTR response queue deleted");
    }
    return ESP_OK;
}

esp_err_t can_expect_rtr_request(uint32_t msg_id, monotonic_timestamp_t timeout_us) {
    if (s_rtr_req_q == NULL) {
        ESP_LOGE(CAN_LOGGER_TAG, "RTR request queue not initialized");
        return ESP_FAIL;
    }

    can_rtr_req_t req = {
        .msg_id = msg_id,
        .timestamp = esp_timer_get_time(),
        .timeout = timeout_us,
    };

    if (xQueueOverwrite(s_rtr_req_q, &req) != pdTRUE) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to overwrite RTR request queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t can_rtr_remove_request() {
    if (s_rtr_req_q == NULL)
        return ESP_FAIL;

    can_rtr_req_t req;
    // Consume the request from the queue -> we could also call xQueueReset() here as we have only one request
    if (xQueueReceive(s_rtr_req_q, &req, NO_WAIT) != pdTRUE) {
        ESP_LOGW(CAN_LOGGER_TAG, "Attempted to remove RTR request, but none was pending.");
        return ESP_FAIL;
    }
    ESP_LOGD(CAN_LOGGER_TAG, "Pending RTR request for ID 0x%03lX removed.", req.msg_id);
    xQueueReset(s_rtr_resp_q);
    return ESP_OK;
}

bool can_rtr_handle_incoming(const twai_message_t *rx_msg, wallclock_timestamp_t wall_time, monotonic_timestamp_t reception_timestamp) {
    if (s_rtr_req_q == NULL || s_rtr_resp_q == NULL)
        return false;

    if (rx_msg->rtr) {
        return false; // RTR responses dont have the rtr flag set -> ignore
    }

    can_rtr_req_t pending_req;
    // Peek to see if a request is pending
    if (xQueuePeek(s_rtr_req_q, &pending_req, NO_WAIT) == pdTRUE) {
        // Check if the incoming message ID matches the pending request ID
        if (pending_req.msg_id == rx_msg->identifier) {
            ESP_LOGD(CAN_LOGGER_TAG, "Received matching RTR response for ID 0x%03lX", rx_msg->identifier);

            xQueueReceive(s_rtr_req_q, &pending_req, NO_WAIT); // Consume it

            can_rtr_resp_t resp;
            resp.msg = *rx_msg;
            resp.timestamp = wall_time; // Wall time of reception
            resp.timed_out = false;
            if (pending_req.timeout > 0) {
                monotonic_timestamp_t elapsed = reception_timestamp - pending_req.timestamp;
                if (elapsed > pending_req.timeout) {
                    resp.timed_out = true;
                    ESP_LOGW(CAN_LOGGER_TAG, "RTR response for ID 0x%03lX arrived late (%llu us > %llu us)", rx_msg->identifier, elapsed, pending_req.timeout);
                }
            }

            if (xQueueOverwrite(s_rtr_resp_q, &resp) != pdTRUE) {
                ESP_LOGE(CAN_LOGGER_TAG, "Failed to overwrite RTR response queue for ID 0x%03lX, message lost", rx_msg->identifier);
            }

            return true;
        }
    }
    return false;
}

esp_err_t can_rtr_get_response(can_rtr_resp_t *out_resp, TickType_t ticks_to_wait) {
    if (s_rtr_resp_q == NULL || out_resp == NULL)
        return ESP_FAIL;

    if (xQueueReceive(s_rtr_resp_q, out_resp, ticks_to_wait) != pdTRUE) {
        // Clear the request if response is not received
        if (can_rtr_remove_request() != ESP_OK) {
            ESP_LOGW(CAN_LOGGER_TAG, "Failed to remove RTR request after timeout");
        }
        return ESP_FAIL;
    }

    ESP_LOGD(CAN_LOGGER_TAG, "Retrieved RTR response for ID 0x%03lX from queue.", out_resp->msg.identifier);
    return ESP_OK;
}