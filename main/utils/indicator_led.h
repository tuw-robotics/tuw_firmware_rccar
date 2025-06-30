#ifndef INDICATOR_LED_H
#define INDICATOR_LED_H

#include <driver/gpio.h>
#include <esp_err.h>

#ifdef CONFIG_ERROR_LED_GPIO
#define ERROR_LED_GPIO CONFIG_ERROR_LED_GPIO
#else
#define ERROR_LED_GPIO 8
#endif

#define INDICATOR_LED_BRIGHTNESS 0.2f // percentage

typedef enum {
    LED_STATUS_OFF = 0,
    LED_STATUS_STARTUP,
    LED_STATUS_OK,
    LED_STATUS_ERROR,
} indicator_led_status_t;

/**
 * @brief Initializes the indicator LED
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t led_indicator_init(void);

/**
 * @brief Sets the status of the indicator LED
 *
 * @param status The status to set the LED to
 */
void indicator_led_set_status(indicator_led_status_t status);

#endif // INDICATOR_LED_H
