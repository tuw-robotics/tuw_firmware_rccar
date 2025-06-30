#include "servo.h"
#include <esp_log.h>

// Map physical [0..180] -> pulsewidth ticks [MIN_PULSE..MAX_PULSE]
static uint32_t angle_to_ticks(int phys_angle) {
    if (phys_angle < SERVO_PHYS_MIN_DEGREE)
        phys_angle = SERVO_PHYS_MIN_DEGREE;
    if (phys_angle > SERVO_PHYS_MAX_DEGREE)
        phys_angle = SERVO_PHYS_MAX_DEGREE;
    return SERVO_MIN_PULSEWIDTH_US + ((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * (uint32_t)phys_angle) / (uint32_t)SERVO_PHYS_MAX_DEGREE;
}

esp_err_t servo_init(servo_t *s, int gpio_num, int8_t zero_offset_deg) {
    // 1) Setup MCPWM timer
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMER_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMER_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    if (mcpwm_new_timer(&timer_cfg, &s->timer) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to create timer");
        return ESP_FAIL;
    }

    // 2) Setup operator
    mcpwm_operator_config_t oper_cfg = {.group_id = 0};
    if (mcpwm_new_operator(&oper_cfg, &s->oper) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to create operator");
        return ESP_FAIL;
    }
    if (mcpwm_operator_connect_timer(s->oper, s->timer) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to connect operator to timer");
        return ESP_FAIL;
    }

    // 3) Setup comparator
    mcpwm_comparator_config_t cmp_cfg = {.flags.update_cmp_on_tez = true};
    if (mcpwm_new_comparator(s->oper, &cmp_cfg, &s->comparator) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to create comparator");
        return ESP_FAIL;
    }

    // 4) Setup generator
    mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = gpio_num};
    if (mcpwm_new_generator(s->oper, &gen_cfg, &s->generator) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to create generator");
        return ESP_FAIL;
    }
    if (mcpwm_generator_set_action_on_timer_event(s->generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to set generator timer event action");
        return ESP_FAIL;
    }
    if (mcpwm_generator_set_action_on_compare_event(s->generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s->comparator, MCPWM_GEN_ACTION_LOW)) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to set generator compare event action");
        return ESP_FAIL;
    }

    // 5) Enable and start timer
    if (mcpwm_timer_enable(s->timer) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to enable timer");
        return ESP_FAIL;
    }
    if (mcpwm_timer_start_stop(s->timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to start timer");
        return ESP_FAIL;
    }

    // 6) Store calibrated zero offset
    if (zero_offset_deg < SERVO_PHYS_MIN_DEGREE)
        zero_offset_deg = SERVO_PHYS_MIN_DEGREE;
    if (zero_offset_deg > SERVO_PHYS_MAX_DEGREE)
        zero_offset_deg = SERVO_PHYS_MAX_DEGREE;
    s->zero_offset_deg = zero_offset_deg;

    // 7) Drive to logical 0 -> physical zero_offset_deg
    uint32_t mid_ticks = angle_to_ticks(s->zero_offset_deg);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s->comparator, mid_ticks));

    ESP_LOGI(SERVO_LOGGER_TAG, "Initialized on GPIO %d: zero at %d° phys", gpio_num, s->zero_offset_deg);
    return ESP_OK;
}

esp_err_t servo_set_angle(servo_t *s, int angle_deg) {
    if (angle_deg < -SERVO_MAX_DEGREE)
        angle_deg = -SERVO_MAX_DEGREE;
    if (angle_deg > SERVO_MAX_DEGREE)
        angle_deg = SERVO_MAX_DEGREE;

    int phys_angle = angle_deg + s->zero_offset_deg;
    if (phys_angle < SERVO_PHYS_MIN_DEGREE)
        phys_angle = SERVO_PHYS_MIN_DEGREE;
    if (phys_angle > SERVO_PHYS_MAX_DEGREE)
        phys_angle = SERVO_PHYS_MAX_DEGREE;

    uint32_t ticks = angle_to_ticks(phys_angle);
    return mcpwm_comparator_set_compare_value(s->comparator, ticks);
}

esp_err_t servo_deinit(servo_t *s) {
    if (mcpwm_del_generator(s->generator) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to delete generator");
        return ESP_FAIL;
    }
    if (mcpwm_del_comparator(s->comparator) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to delete comparator");
        return ESP_FAIL;
    }
    if (mcpwm_del_operator(s->oper) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to delete operator");
        return ESP_FAIL;
    }
    if (mcpwm_del_timer(s->timer) != ESP_OK) {
        ESP_LOGE(SERVO_LOGGER_TAG, "Failed to delete timer");
        return ESP_FAIL;
    }
    return ESP_OK;
}
