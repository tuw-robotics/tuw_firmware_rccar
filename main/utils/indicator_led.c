#include "indicator_led.h"
#include <esp_log.h>
#include <led_strip.h>

static const char *TAG = "INDICATOR_LED";
static led_strip_handle_t led_strip = NULL;

esp_err_t led_indicator_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = ERROR_LED_GPIO,
        .max_leds = 1, // One RGB LED
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(err));
        return err;
    }

    led_strip_clear(led_strip);
    return ESP_OK;
}

void indicator_led_set_status(indicator_led_status_t status) {
    if (!led_strip) {
        return;
    }

    uint8_t r = 0, g = 0, b = 0;

    switch (status) {
    case LED_STATUS_OFF:
        r = g = b = 0;
        break;
    case LED_STATUS_STARTUP: // Yellow //! Note a nice yellow color...
        r = 255;
        g = 255;
        b = 0;
        break;
    case LED_STATUS_OK: // Green
        r = 0;
        g = 255;
        b = 0;
        break;
    case LED_STATUS_ERROR: // Red
        r = 255;
        g = 0;
        b = 0;
        break;
    }

    r = (uint8_t)(r * INDICATOR_LED_BRIGHTNESS);
    g = (uint8_t)(g * INDICATOR_LED_BRIGHTNESS);
    b = (uint8_t)(b * INDICATOR_LED_BRIGHTNESS);

    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}
