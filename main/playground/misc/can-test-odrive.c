#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#define TWAI_TX_GPIO GPIO_NUM_23
#define TWAI_RX_GPIO GPIO_NUM_22

// ODrive CAN configuration: Node ID is now set to 0
#define NODE_ID 0
#define CMD_GET_BUS_VOLTAGE 0x17
#define GET_BUS_VOLTAGE_MSG_ID ((NODE_ID << 5) | CMD_GET_BUS_VOLTAGE)
// Since NODE_ID_M0 == 0, GET_BUS_VOLTAGE_MSG_ID is 0x17

#define EXAMPLE_TAG "TWAI GET_BUS_VOLTAGE"

// TWAI timing configuration for 250 kbps
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

// Filter configuration: accept only messages with GET_BUS_VOLTAGE_MSG_ID
static const twai_filter_config_t f_config = {.acceptance_code = (GET_BUS_VOLTAGE_MSG_ID << 21), .acceptance_mask = ~(TWAI_STD_ID_MASK << 21), .single_filter = true};

// General TWAI configuration in NORMAL mode
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

void app_main(void) {
    // Install and start the TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(LOGGER_TAG, "TWAI driver started");

    // Allow a brief delay for the driver to initialize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create a remote transmission request for Get_Bus_Voltage_Current:
    // Use RTR=1 (remote frame) with DLC set to 8 bytes.
    twai_message_t tx_msg = {
        .extd = 0, // Use standard 11-bit identifier
        .rtr = 1,  // Remote frame request
        .ss = 0,
        .self = 0,
        .identifier = GET_BUS_VOLTAGE_MSG_ID,
        .data_length_code = 8, // Expect an 8-byte response
        .data = {0}            // Payload not used for remote frames
    };

    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(LOGGER_TAG, "Failed to transmit remote frame: %d", err);
    }

    // Wait for the response from the ODrive (blocking call)
    twai_message_t rx_msg;
    esp_err_t rx_err = twai_receive(&rx_msg, portMAX_DELAY);
    if (rx_err != ESP_OK) {
        ESP_LOGE(LOGGER_TAG, "Failed to receive response: %d", rx_err);
    }

    ESP_LOGI(LOGGER_TAG, "Received message: ID=%d, DLC=%d", (int)rx_msg.identifier, (int)rx_msg.data_length_code);

    // The Get_Bus_Voltage_Current response should have 8 data bytes:
    //  - Bytes 0-3: Bus Voltage (float32, in Volts)
    //  - Bytes 4-7: Bus Current (float32, in Amps)
    if (rx_msg.identifier == GET_BUS_VOLTAGE_MSG_ID && rx_msg.data_length_code == 8) {
        float bus_voltage, bus_current;
        memcpy(&bus_voltage, rx_msg.data, sizeof(float));
        memcpy(&bus_current, rx_msg.data + 4, sizeof(float));

        ESP_LOGI(LOGGER_TAG, "ODrive Bus Voltage: %.2f V", bus_voltage);
        ESP_LOGI(LOGGER_TAG, "ODrive Bus Current: %.2f A", bus_current);
    } else {
        ESP_LOGE(LOGGER_TAG, "Received unexpected message!");
    }

    // Delay briefly to yield
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(LOGGER_TAG, "TWAI driver stopped");
    vTaskDelete(NULL);
}
