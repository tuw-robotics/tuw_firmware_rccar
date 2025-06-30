#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "odrive_enums.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// TODO: Use similar TWAI handling as in "https://github.com/espressif/arduino-esp32/issues/9191"

#define LOGGER_TAG "ODrive CAN Test"

//? const <type> should be prefered over #define as it is type safe
// TODO: Replace the constants that should be configurable with kconfig values
const uint32_t CAN_TX_GPIO = GPIO_NUM_23;
const uint32_t CAN_RX_GPIO = GPIO_NUM_22;

//! Odrive must be configured to have a node ID of 0
const uint8_t NODE_ID_M0 = 0; // -> 11'000000 XXXXX
                              // first 6 bits are node id, later 5 are command id
const uint8_t NODE_ID_M1 = 1; // -> 11'000001 XXXXX
// -> acceptance mask for those two:11'000001 00000

typedef enum {
    STATE_INIT,    // TODO: Split this into multiple states
    STATE_RUNNING, // This also
    STATE_ERROR
} control_state_t;

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); //! If the system grows, this should be changed to only accept the messages we want

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

void app_main(void) {
    control_state_t state = STATE_INIT;
    bool should_run = true;

    while (should_run) {
        switch (state) {
        case STATE_INIT: {
            // --- TWAI INIT ---
            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
            ESP_ERROR_CHECK(twai_start());
            ESP_LOGI(LOGGER_TAG, "TWAI driver started");

            // TODO: Setup alerts: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html#_CPPv416twai_read_alertsP8uint32_t10TickType_t
            // Use Alerts (https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html#alerts)
            // Setup an interrupt that sets the state machine of the task from idle into ?running? if a alert is received (like RX NE)

            uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_IDLE;
            if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
                ESP_LOGI(LOGGER_TAG, "TWAI alerts configured");
            } else {
                ESP_LOGE(LOGGER_TAG, "TWAI alerts configuration failed");
                state = STATE_ERROR;
                break;
            }

            // Flush rx queue
            esp_err_t flush_err = twai_clear_receive_queue();
            if (flush_err != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "Failed to receive response: %d", (int)flush_err);
            }
            ESP_LOGI(LOGGER_TAG, "TWAI queue cleared");

            // --- ODRIVE CHECKS ----

            // Check if heartbeats are being received
            // TODO: Heartbeats can be switched off, instead use Discovery (https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#discovery-addressing)
            // Enumerate all ODrives on the bus:
            // Send (node_id = 0x3f, cmd_id = 0x06, RTR=1).
            // All ODrives that are already addressed (node_id != 0x3f) will respond immediately with their serial number and node_id.
            // All unaddressed ODrives (node_id == 0x3f) will respond after a small random delay, averaging 250 ms between each message.
            twai_message_t twai_msg;
            uint32_t error;
            uint8_t axis_state, result, traj_done;
            // Wait for first heartbeat message
            // TODO: Move this to separate state -> so that it is no longer a blocking while. Also: only call twai receive if reception alert is set
            do {
                esp_err_t rx_err = twai_receive(&twai_msg, 0); //! Non blocking -> only polling
                if (rx_err != ESP_OK) {                        // TODO: Pack that into the ESP_ERROR_CHECK macro
                    ESP_LOGE(LOGGER_TAG, "Failed to receive response: %d", (int)rx_err);
                }
            } while (twai_msg.identifier != can_get_msg_id(NODE_ID_M0, CMD_HEARTBEAT) || twai_msg.data_length_code != 8);
            ESP_LOGI(LOGGER_TAG, "Odrive detected through heartbeat");

            // Unpack heartbeat message into outer variables
            odrive_heartbeat_t heartbeat;
            esp_err_t err = can_unpack_message(&twai_msg, &heartbeat, sizeof(odrive_heartbeat_t), can_get_msg_id(NODE_ID_M0, CMD_HEARTBEAT));

            if (error != ODRIVE_ERROR_NONE) {
                ESP_LOGE(LOGGER_TAG, "Failed to transmit remote frame: %d", (int)error);
                state = STATE_ERROR; // update the control state
                break;
            }

            // Wait until the axis state leaves STARTUP_SEQUENCE
            // TODO: Move this to separate state -> so that it is no longer a blocking while. Also: only call twai receive if reception alert is set
            while ((int)axis_state == AXIS_STATE_STARTUP_SEQUENCE) {
                do {
                    esp_err_t rx_err = twai_receive(&twai_msg, 0);
                    if (rx_err != ESP_OK) {
                        ESP_LOGE(LOGGER_TAG, "Failed to receive response: %d", (int)rx_err);
                    }
                } while (twai_msg.identifier !=
                             can_get_msg_id(NODE_ID_M0, CMD_HEARTBEAT) || // this checks if actually a heartbeat message is received // TODO: How should the other message receptions be handled?
                         twai_msg.data_length_code != 8);

                // Update heartbeat variables
                memcpy(&error, twai_msg.data, sizeof(uint32_t));
                memcpy(&axis_state, twai_msg.data + sizeof(uint32_t), sizeof(uint8_t));
                memcpy(&result, twai_msg.data + sizeof(uint32_t) + sizeof(uint8_t), sizeof(uint8_t));
                memcpy(&traj_done, twai_msg.data + sizeof(uint32_t) + 2 * sizeof(uint8_t), sizeof(uint8_t));
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            if (axis_state != AXIS_STATE_IDLE) { // TODO: Set this also as separate state
                // Set AXIS_STATE_IDLE
                twai_message_t tx_msg_2 = {.extd = 0, // Use standard 11-bit identifier
                                           .rtr = 0,  // Data frame (not a remote frame request)
                                           .ss = 0,
                                           .self = 0,
                                           .identifier = can_get_msg_id(NODE_ID_M0, CMD_SET_AXIS_STATE),
                                           .data_length_code = sizeof(uint32_t),
                                           .data = {AXIS_STATE_IDLE}};

                //! The TX_IDLE alert can be used to alert the application when no other messages are awaiting transmission.
                // ESP_OK: Transmission successfully queued/initiated

                // ESP_ERR_INVALID_ARG: Arguments are invalid

                // ESP_ERR_TIMEOUT: Timed out waiting for space on TX queue

                // ESP_FAIL: TX queue is disabled and another message is currently transmitting

                // ESP_ERR_INVALID_STATE: TWAI driver is not in running state, or is not installed

                // ESP_ERR_NOT_SUPPORTED: Listen Only Mode does not support transmissions
                esp_err_t err = twai_transmit(&tx_msg_2, pdMS_TO_TICKS(1000)); // TODO: Here also use alerts
                if (err != ESP_OK) {
                    ESP_LOGE(LOGGER_TAG, "Failed to transmit remote frame: %d", (int)err);
                }

                // Wait for IDLE
                while ((int)axis_state != AXIS_STATE_IDLE) {
                    do {
                        //? https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html#_CPPv412twai_receiveP14twai_message_t10TickType_t
                        esp_err_t rx_err = twai_receive(&twai_msg, pdMS_TO_TICKS(1000));
                        if (rx_err != ESP_OK) {
                            ESP_LOGE(LOGGER_TAG, "Failed to receive response: %d", (int)rx_err);
                        }
                    } while (twai_msg.identifier != can_get_msg_id(NODE_ID_M0, CMD_HEARTBEAT) || twai_msg.data_length_code != 8);
                    memcpy(&error, twai_msg.data, sizeof(uint32_t));
                    memcpy(&axis_state, twai_msg.data + sizeof(uint32_t), sizeof(uint8_t));
                    memcpy(&result, twai_msg.data + sizeof(uint32_t) + sizeof(uint8_t), sizeof(uint8_t));
                    memcpy(&traj_done, twai_msg.data + sizeof(uint32_t) + 2 * sizeof(uint8_t), sizeof(uint8_t));
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            // Prepare payload with two uint32_t values that correspond to:
            // - Closed loop control state
            // - Input mode pos filter
            uint32_t payload[2] = {CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_POS_FILTER};

            twai_message_t tx_msg = {.extd = 0, // Use standard 11-bit identifier
                                     .rtr = 0,  // Data frame (not a remote frame request)
                                     .ss = 0,
                                     .self = 0,
                                     .identifier = can_get_msg_id(NODE_ID_M0, CMD_SET_CONTROL_MODE),
                                     .data_length_code = sizeof(payload)};
            memcpy(tx_msg.data, payload, sizeof(payload));

            esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
            if (err != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "Failed to transmit remote frame: %d", (int)err);
            }
            ESP_LOGI(LOGGER_TAG, "Control mode set to POSITION_CONTROL");

            // Set AXIS_STATE_CLOSED_LOOP_CONTROL
            twai_message_t tx_msg_2 = {.extd = 0, // Use standard 11-bit identifier
                                       .rtr = 0,  // Data frame (not a remote frame request)
                                       .ss = 0,
                                       .self = 0,
                                       .identifier = can_get_msg_id(NODE_ID_M0, CMD_SET_AXIS_STATE),
                                       .data_length_code = sizeof(uint32_t),
                                       .data = {AXIS_STATE_CLOSED_LOOP_CONTROL}};
            err = twai_transmit(&tx_msg_2, pdMS_TO_TICKS(1000));
            if (err != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "Failed to transmit remote frame: %d", (int)err);
            }

            // Wait until AXIS_STATE_CLOSED_LOOP_CONTROL
            while ((int)axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) { // TODO: Add timeout -> actually use the state machine instead
                do {
                    esp_err_t rx_err = twai_receive(&twai_msg, pdMS_TO_TICKS(1000));
                    if (rx_err != ESP_OK) {
                        ESP_LOGE(LOGGER_TAG, "Failed to receive response: %d", (int)rx_err);
                    }
                    ESP_LOGI(LOGGER_TAG, "Received message with id: %d", (int)twai_msg.identifier);
                } while (twai_msg.identifier != can_get_msg_id(NODE_ID_M0, CMD_HEARTBEAT) || twai_msg.data_length_code != 8);
                memcpy(&error, twai_msg.data, sizeof(uint32_t));
                memcpy(&axis_state, twai_msg.data + sizeof(uint32_t), sizeof(uint8_t));
                memcpy(&result, twai_msg.data + sizeof(uint32_t) + sizeof(uint8_t), sizeof(uint8_t));
                memcpy(&traj_done, twai_msg.data + sizeof(uint32_t) + 2 * sizeof(uint8_t), sizeof(uint8_t));
                ESP_LOGI(LOGGER_TAG, "Axis State currently: %d", (int)axis_state);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            ESP_LOGI(LOGGER_TAG, "Axis State is now CLOSED_LOOP_CONTROL");
            state = STATE_RUNNING;
            break;
        }
        case STATE_RUNNING: {

            // Here:
            // - (Check if odrive connection is still there, otherwise go into error state)
            // - Read from setpoint queue (if new vel setpoints are there)
            // - If new setpoints are there, send them to the odrive
            // - Always read measured values for (currently only) velocity and put them into queue //! No need to send get messages to the odrive for this, as they can be set to automatically be sent
            // as cyclic messages
            //      ? https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#cyclic-messages
            //      ! Queues always need timestamps!!
            // - (check if emergency stop signal is set, if so, immediatly send to odrive (maybe with increased priority and clearing of previous transmissions in buffer))

            uint32_t alerts_triggered;
            twai_read_alerts(&alerts_triggered, 0); // 0 results in non-blocking -> only polling //! This should be used in running mode -> only read from queue if alert for TWAI_ALERT_RX_DATA is set
                                                    //! -> Only send if TWAI_ALERT_TX_IDLE
                                                    //? Should other alerts also be used?
            // Running state: send sinusoidal setpoints for 10 seconds.
            double t0 = esp_timer_get_time() / 1000000.0; // initial time in seconds
            while (((esp_timer_get_time() / 1000000.0) - t0) <
                   10.0) { //! <- this also needs to be adjusted -> busy waiting -> instead obviously the statemachine should be used where a vtaskdelayuntil should be added
                double current_time = esp_timer_get_time() / 1000000.0;
                float setpoint = 4.0f * sinf((current_time - t0) * 2.0f);
                float desired_vel = 8.0f * cosf((current_time - t0) * 2.0f);
                int16_t ff_vel = (int16_t)desired_vel;
                int16_t ff_torque = 0;
                uint8_t data[8];
                memcpy(data, &setpoint, sizeof(setpoint));
                memcpy(data + sizeof(setpoint), &ff_vel, sizeof(ff_vel));
                memcpy(data + sizeof(setpoint) + sizeof(ff_vel), &ff_torque, sizeof(ff_torque));

                twai_message_t sine_msg = {.extd = 0, .rtr = 0, .ss = 0, .self = 0, .identifier = can_get_msg_id(NODE_ID_M0, CMD_SET_INPUT_POS), .data_length_code = 8};
                memcpy(sine_msg.data, data, 8);

                esp_err_t tx_err = twai_transmit(&sine_msg, pdMS_TO_TICKS(100));
                if (tx_err != ESP_OK) {
                    ESP_LOGE(LOGGER_TAG, "Error transmitting sine wave message: %d", (int)tx_err);
                }

                // Check for incoming encoder feedback messages with a short timeout.
                twai_message_t rx_msg;
                esp_err_t rx_err = twai_receive(&rx_msg, pdMS_TO_TICKS(1));
                if (rx_err == ESP_OK && rx_msg.identifier == can_get_msg_id(NODE_ID_M0, CMD_GET_ENCODER_ESTIMATES)) {
                    if (rx_msg.data_length_code >= 8) {
                        float encoder_pos, encoder_vel;
                        memcpy(&encoder_pos, rx_msg.data, sizeof(float));
                        memcpy(&encoder_vel, rx_msg.data + sizeof(float), sizeof(float));
                        ESP_LOGI(LOGGER_TAG, "Encoder feedback: pos: %.3f [turns], vel: %.3f [turns/s]", encoder_pos, encoder_vel);
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(20));
            }
            ESP_LOGI(LOGGER_TAG, "Exiting RUNNING STATE after 10 seconds");
            should_run = false;
            break;
        }
        case STATE_ERROR:
            // TODO: If first time in error state, try sending clear error message to odrive and go back to init state
            should_run = false;
            break;
        }
    }

    // Set axis state back to idle
    twai_message_t tx_msg = {.extd = 0, // Use standard 11-bit identifier
                             .rtr = 0,  // Data frame (not a remote frame request)
                             .ss = 0,
                             .self = 0,
                             .identifier = can_get_msg_id(NODE_ID_M0, CMD_SET_AXIS_STATE),
                             .data_length_code = sizeof(uint32_t),
                             .data = {AXIS_STATE_IDLE}};

    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(LOGGER_TAG, "Failed to transmit remote frame: %d", (int)err);
    }

    // Wait until AXIS_STATE_IDLE
    uint8_t idle_axis_state = AXIS_STATE_UNDEFINED;
    twai_message_t heartbeat_msg;
    while ((int)idle_axis_state != AXIS_STATE_IDLE) { // TODO: Add timeout
        do {
            esp_err_t rx_err = twai_receive(&heartbeat_msg, pdMS_TO_TICKS(1000));
            if (rx_err != ESP_OK) {
                ESP_LOGE(LOGGER_TAG, "Failed to receive response: %d", (int)rx_err);
            }
        } while (heartbeat_msg.identifier != can_get_msg_id(NODE_ID_M0, CMD_HEARTBEAT) || heartbeat_msg.data_length_code != 8);
        memcpy(&idle_axis_state, heartbeat_msg.data + sizeof(uint32_t), sizeof(uint8_t));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(LOGGER_TAG, "TWAI driver stopped");
    vTaskDelete(NULL);
}
