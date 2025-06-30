/*
 * Receiver ESP32: Receives TWAI messages and sets an LED (GPIO25) based on the message.
 * If the received data byte is 1, the LED is turned ON; if 0, it is turned OFF.
 */

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define TWAI_TX_GPIO GPIO_NUM_32
#define TWAI_RX_GPIO GPIO_NUM_33
#define LED_GPIO_NUM GPIO_NUM_25
#define MSG_ID 0x555
#define EXAMPLE_TAG "TWAI RECEIVER"

// TWAI timing configuration (25 Kbps)
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();

// Filter configuration to accept only messages with MSG_ID
static const twai_filter_config_t f_config = {.acceptance_code = (MSG_ID << 21), .acceptance_mask = ~(TWAI_STD_ID_MASK << 21), .single_filter = true};

// Use NORMAL mode for communication (not NO_ACK/self-test)
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

void app_main(void) {
    // Configure LED pin (GPIO25) as an output
    gpio_reset_pin(LED_GPIO_NUM);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO_NUM, GPIO_MODE_OUTPUT);

    // Install and start the TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(LOGGER_TAG, "TWAI driver started");

    while (1) {
        twai_message_t rx_msg;

        // Wait indefinitely for a TWAI message
        ESP_ERROR_CHECK(twai_receive(&rx_msg, portMAX_DELAY));
        ESP_LOGI(LOGGER_TAG, "Received message: Data = %d", rx_msg.data[0]);

        // Set LED state based on the received data (1 = ON, 0 = OFF)
        if (rx_msg.data[0] == 1) {
            gpio_set_level(LED_GPIO_NUM, 1);
            ESP_LOGI(LOGGER_TAG, "LED ON");
        } else if (rx_msg.data[0] == 0) {
            gpio_set_level(LED_GPIO_NUM, 0);
            ESP_LOGI(LOGGER_TAG, "LED OFF");
        } else {
            ESP_LOGW(LOGGER_TAG, "Unexpected data value: %d", rx_msg.data[0]);
        }
    }

    // (Unreachable cleanup code)
    // ESP_ERROR_CHECK(twai_stop());
    // ESP_ERROR_CHECK(twai_driver_uninstall());
}
