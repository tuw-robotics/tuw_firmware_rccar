#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <driver/twai.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the CAN driver with default configuration
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 * @note Defaults to 250kbps and RX/TX GPIOs defined in sdkconfig, accepting all messages
 */
esp_err_t can_driver_init_default(void);

/**
 * @brief Initialize the CAN driver with custom configuration
 *
 * @param g_config Configuration for the general settings of the TWAI driver
 * @param t_config Configuration for the timing settings of the TWAI driver
 * @param f_config Configuration for the filter settings of the TWAI driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_driver_init(const twai_general_config_t *g_config, const twai_timing_config_t *t_config, const twai_filter_config_t *f_config);

/**
 * @brief Start the CAN driver
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_driver_start(void);

/**
 * @brief Deinitialize the CAN driver
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_driver_deinit(void);

/**
 * @brief Stop the CAN driver
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t can_driver_stop(void);

/**
 * @brief Check if the CAN driver is running
 *
 * @return true if the driver is running, false otherwise
 */
bool can_driver_is_running(void);

/**
 * @brief Check if the CAN driver is in bus-off state
 *
 * @return true if the successful, false otherwise
 */
esp_err_t can_clear_receive_queue(void);

/**
 * @brief Transmit a CAN message
 *
 * @param msg Pointer to the CAN message to be transmitted
 * @param ticks_to_wait Maximum time to wait for the message to be transmitted
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if msg is NULL, ESP_FAIL if the message could not be transmitted within the specified time
 */
esp_err_t can_driver_transmit(const twai_message_t *msg, TickType_t ticks_to_wait);

/**
 * @brief Receive a CAN message
 *
 * @param msg Pointer to the CAN message structure to store the received message
 * @param ticks_to_wait Maximum time to wait for a message to be received
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if msg is NULL, ESP_FAIL if the message could not be received within the specified time
 */
esp_err_t can_driver_receive(twai_message_t *msg, TickType_t ticks_to_wait);

#endif // CAN_DRIVER_H