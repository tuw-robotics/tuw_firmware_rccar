/*
 * Sender ESP32: Reads a serial input (0 or 1) and sends it over TWAI.
 * This version sets STDIN to non-blocking mode so that the task can yield
 * periodically and avoid triggering the task watchdog.
 */

#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <fcntl.h> // for fcntl() to set non-blocking mode
#include <stdio.h>
#include <sys/unistd.h>

#define TWAI_TX_GPIO GPIO_NUM_32
#define TWAI_RX_GPIO GPIO_NUM_33
#define MSG_ID 0x555
#define EXAMPLE_TAG "TWAI SENDER"

// TWAI timing configuration (25 Kbps)
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
// TODO: For Odrive, use TWAI_TIMING_CONFIG_250KBITS()

// Filter configuration to accept only messages with MSG_ID
static const twai_filter_config_t f_config = {.acceptance_code = (MSG_ID << 21), .acceptance_mask = ~(TWAI_STD_ID_MASK << 21), .single_filter = true};

// Use NORMAL mode for communication (not NO_ACK/self-test)
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

void app_main(void) {
    // Install and start the TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(LOGGER_TAG, "TWAI driver started");

    // Set STDIN to non-blocking mode so that getchar() does not block forever.
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(LOGGER_TAG, "Enter 0 or 1:");

    while (1) {
        int c = getchar();
        if (c != EOF) {
            // Ignore newline or carriage return characters
            if (c == '\n' || c == '\r') {
                // Do nothing
            } else if (c == '0' || c == '1') {
                uint8_t data_value = (c == '1') ? 1 : 0;
                twai_message_t tx_msg = {.extd = 0, // Standard 11-bit identifier
                                         .rtr = 0,  // Data frame (not a remote frame)
                                         .ss = 0,   // Not single shot
                                         .self = 0, // Normal transmission (not self reception)
                                         .identifier = MSG_ID,
                                         .data_length_code = 1, // One data byte
                                         .data = {data_value}};

                // Transmit the message (wait up to 1000 ms)
                ESP_ERROR_CHECK(twai_transmit(&tx_msg, pdMS_TO_TICKS(1000)));
                ESP_LOGI(LOGGER_TAG, "Message transmitted: %c", c);
            } else {
                ESP_LOGW(LOGGER_TAG, "Invalid input. Please enter 0 or 1.");
            }
        }
        // Delay to yield to other tasks and prevent watchdog timeout.
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // (Unreachable cleanup code)
    // ESP_ERROR_CHECK(twai_stop());
    // ESP_ERROR_CHECK(twai_driver_uninstall());
}
