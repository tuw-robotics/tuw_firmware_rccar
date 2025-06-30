#include "jtag_debug.h"

#include <driver/usb_serial_jtag.h>
#include <esp_err.h>

esp_err_t jtag_debug_init(void) {
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = JTAG_BUFF_SIZE,
        .tx_buffer_size = JTAG_BUFF_SIZE,
    };

    if (usb_serial_jtag_driver_install(&usb_serial_jtag_config) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}