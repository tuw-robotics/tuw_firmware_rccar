#ifndef CAN_RTR_H
#define CAN_RTR_H

#include "utils/timing_utils.h"
#include <driver/twai.h>
#include <esp_err.h>
#include <freertos/queue.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t msg_id;
    monotonic_timestamp_t timestamp;
    monotonic_timestamp_t timeout;
} can_rtr_req_t;

typedef struct {
    twai_message_t msg;
    wallclock_timestamp_t timestamp;
    bool timed_out;
} can_rtr_resp_t;

/**
 * @brief Initializes the RTR request and response queues
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 * @note This function should be called before using any RTR functionality
 */
esp_err_t can_rtr_init(void);

/**
 * @brief Deinitializes the RTR request and response queues
 *
 * @return esp_err_t Always returns ESP_OK
 */
esp_err_t can_rtr_deinit(void);

/**
 * @brief Requests an RTR message with the specified ID and timeout
 *
 * @param msg_id The ID of the message to request
 * @param timeout_us The timeout in microseconds for the request
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 * @note This function will overwrite any previous pending request
 */
esp_err_t can_expect_rtr_request(uint32_t msg_id, monotonic_timestamp_t timeout_us);

/**
 * @brief Removes the current RTR request from the queue
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_rtr_remove_request(void);

/**
 * @brief Handles incoming RTR responses. This function should be called from the CAN dispatcher task
 *
 * @param rx_msg The received message to be processed
 * @param wall_time The wallclock timestamp of the reception
 * @param reception_timestamp The monotonic timestamp of the reception
 * @return true if the message was handled successfully and should not be dispatched further
 * @return false if the message was not an RTR response or if it was not handled
 */
bool can_rtr_handle_incoming(const twai_message_t *rx_msg, wallclock_timestamp_t wall_time, monotonic_timestamp_t reception_timestamp);

/**
 * @brief Gets the response for the last RTR request
 *
 * @param out_resp Pointer to the structure where the response will be stored
 * @param ticks_to_wait The maximum time to wait for the response
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_rtr_get_response(can_rtr_resp_t *out_resp, TickType_t ticks_to_wait);

#endif // CAN_RTR_H