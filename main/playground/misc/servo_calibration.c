#include "servo/servo.h"
#include "utils/indicator_led.h"
#include "utils/math_utils.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#define MAIN_TAG "MAIN"
#define SERVO_GPIO GPIO_NUM_5
#define UART_NUM UART_NUM_0
#define UART_BUF_SIZE 256

void app_main(void) {
    // --- 1) Indicator LED init ---
    if (led_indicator_init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize error indicator");
        goto error_handling;
    }
    indicator_led_set_status(LED_STATUS_STARTUP);
    ESP_LOGI(MAIN_TAG, "Error indicator initialized successfully");

    // --- 2) Servo init ---
    servo_t servo;
    if (servo_init(&servo, SERVO_GPIO, 46) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize servo");
        goto error_handling;
    }
    ESP_LOGI(MAIN_TAG, "Servo initialized successfully");

    indicator_led_set_status(LED_STATUS_OK);
    ESP_LOGI(MAIN_TAG, "Startup sequence completed successfully");

    // --- 3) UART init (for console input) ---
    const uart_config_t uart_config = {.baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0));

    // --- 4) Interactive loop variables ---
    float angle_deg = 0.0f;
    const float step = 1.0f; // 1° per button press
    uint8_t data;

    // Center servo at 0°
    ESP_LOGI(MAIN_TAG, "Centering servo at %0.1f°", angle_deg);
    ESP_ERROR_CHECK(servo_set_angle(&servo, angle_deg));

    printf("\nServo calibration console:\n");
    printf("  '+' → +%0.1f°\n", step);
    printf("  '-' → -%0.1f°\n", step);
    printf("  'r' → reset to  0°\n\n");
    printf("Current angle: %0.1f°\n", angle_deg);

    // --- 5) Main loop: read serial, update servo ---
    while (true) {
        int len = uart_read_bytes(UART_NUM, &data, 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            switch (data) {
            case '+':
                angle_deg += step;
                break;
            case '-':
                angle_deg -= step;
                break;
            case 'r':
                angle_deg = 0.0f;
                break;
            default:
                continue; // ignore other chars
            }

            ESP_LOGI(MAIN_TAG, "New angle: %0.1f°", angle_deg);
            ESP_ERROR_CHECK(servo_set_angle(&servo, angle_deg));
            printf("Current angle: %0.1f°\n", angle_deg);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

error_handling:
    indicator_led_set_status(LED_STATUS_ERROR);
    servo_deinit(&servo);
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(MAIN_TAG, "Restarting...");
    esp_restart();
}
