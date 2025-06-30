#include "can_driver.h"
#include "can_conf.h"

#include <esp_log.h>
#include <string.h>

esp_err_t can_driver_init_default() {
    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_GPIO, (gpio_num_t)CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g_cfg.rx_queue_len = CAN_RX_QUEUE_LEN;
    g_cfg.tx_queue_len = CAN_TX_QUEUE_LEN;
    g_cfg.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF;

    const twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_250KBITS();
    const twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    return can_driver_init(&g_cfg, &t_cfg, &f_cfg);
}

esp_err_t can_driver_init(const twai_general_config_t *g_cfg, const twai_timing_config_t *t_cfg, const twai_filter_config_t *f_cfg) {
    if (twai_driver_install(g_cfg, t_cfg, f_cfg) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to install TWAI driver");
        return ESP_FAIL;
    }
    ESP_LOGI(CAN_LOGGER_TAG, "TWAI driver installed");

    return ESP_OK;
}

esp_err_t can_driver_start(void) {
    if (twai_start() != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to start TWAI driver");
        return ESP_FAIL;
    }
    ESP_LOGI(CAN_LOGGER_TAG, "TWAI driver started");
    return ESP_OK;
}

esp_err_t can_driver_deinit(void) {
    if (can_driver_stop() != ESP_OK) {
        ESP_LOGW(CAN_LOGGER_TAG, "Failed to stop CAN driver, it might already be stopped");
    }
    if (twai_driver_uninstall() != ESP_OK) {
        ESP_LOGW(CAN_LOGGER_TAG, "Failed to uninstall TWAI driver, it might already be uninstalled");
    }
    return ESP_OK;
}

esp_err_t can_driver_stop(void) {
    if (twai_stop() != ESP_OK) {
        ESP_LOGW(CAN_LOGGER_TAG, "Failed to stop TWAI driver, it might already be stopped");
    }
    return ESP_OK;
}

bool can_driver_is_running(void) {
    twai_status_info_t status;

    if (twai_get_status_info(&status) != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to get TWAI driver state");
        return false;
    }
    return (status.state == TWAI_STATE_RUNNING);
}

esp_err_t can_clear_receive_queue(void) {
    if (twai_clear_receive_queue() != ESP_OK) {
        ESP_LOGE(CAN_LOGGER_TAG, "Failed to clear TWAI receive queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t can_driver_transmit(const twai_message_t *msg, TickType_t ticks_to_wait) {
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (twai_transmit(msg, ticks_to_wait) != ESP_OK) {
        ESP_LOGW(CAN_LOGGER_TAG, "Failed to transmit CAN message (ID: 0x%03lX, RTR: %d)", msg->identifier, msg->rtr);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t can_driver_receive(twai_message_t *msg, TickType_t ticks_to_wait) {
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // No need to log success here, dispatcher will handle received messages
    return twai_receive(msg, ticks_to_wait);
}