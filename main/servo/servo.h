#ifndef SERVO_H
#define SERVO_H

#include <driver/mcpwm_prelude.h>
#include <esp_err.h>
#include <stdint.h>

//! https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html
//! Was not up to date during the time of writing,
//! This code is based on https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_servo_control/main/mcpwm_servo_control_example_main.c

#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500

// PHYSICAL sweep of the servo
#define SERVO_PHYS_MIN_DEGREE 0
#define SERVO_PHYS_MAX_DEGREE 180

// LOGICAL half-range from the zero point
#define SERVO_MAX_DEGREE 45

#define SERVO_PWM_FREQUENCY_HZ 50
#define SERVO_LOGGER_TAG "SERVO"

// 1 MHz timebase -> 1 tick = 1 µs
#define SERVO_TIMER_RESOLUTION_HZ (1000000U)
#define SERVO_TIMER_PERIOD_TICKS (SERVO_TIMER_RESOLUTION_HZ / SERVO_PWM_FREQUENCY_HZ)

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;

    int8_t zero_offset_deg; //! the calibrated “middle” in physical degrees
} servo_t;

/**
 * @brief Initializes a servo motor on a specified GPIO pin
 *
 * @param s Pointer to the servo_t structure to initialize
 * @param gpio_num GPIO number to which the servo is connected
 * @param zero_offset_deg Calibrated zero offset in degrees, which is the physical position of the servo when it is at logical 0
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t servo_init(servo_t *s, int gpio_num, int8_t zero_offset_deg);

/**
 * @brief Sets the angle of the servo motor
 *
 * @param s Pointer to the servo_t structure
 * @param angle_deg Angle in degrees to set the servo to, within the range [-SERVO_MAX_DEGREE ... +SERVO_MAX_DEGREE]
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t servo_set_angle(servo_t *s, int angle_deg);

/**
 * @brief Deinitializes the servo motor and releases resources
 *
 * @param s Pointer to the servo_t structure to deinitialize
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t servo_deinit(servo_t *s);

#endif // SERVO_H
