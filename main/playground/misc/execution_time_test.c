#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <inttypes.h>
#include <stdint.h>

static const char *TAG = "fast_control_demo";

#define FAST_TASK_GPIO GPIO_NUM_33 // Channel 1 of Oscilloscope
#define SLOW_TASK_GPIO GPIO_NUM_32 // Channel 2

static QueueHandle_t dataQueue = NULL;

volatile uint32_t fastCounter = 0;

#ifdef ESP_TIMER_ISR
#define TIMER_DISPATCH_METHOD ESP_TIMER_ISR
#else
#define TIMER_DISPATCH_METHOD ESP_TIMER_TASK
#endif

void IRAM_ATTR fastControlCallback(void *arg) {
    gpio_set_level(FAST_TASK_GPIO, 1);

    uint32_t value = fastCounter++;

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xQueueOverwriteFromISR(dataQueue, &value, &higherPriorityTaskWoken);

    gpio_set_level(FAST_TASK_GPIO, 0);

    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void slowTask(void *pvParameters) {
    gpio_set_level(SLOW_TASK_GPIO, 0);
    uint32_t receivedValue;
    while (1) {
        if (xQueueReceive(dataQueue, &receivedValue, portMAX_DELAY) == pdPASS) {
            gpio_set_level(SLOW_TASK_GPIO, 1);

            ESP_LOGI(TAG, "Slow Task received: %" PRIu32, receivedValue);

            vTaskDelay(pdMS_TO_TICKS(100));

            gpio_set_level(SLOW_TASK_GPIO, 0);
        }
    }
}

void app_main(void) {
    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << FAST_TASK_GPIO) | (1ULL << SLOW_TASK_GPIO),
                             .mode = GPIO_MODE_OUTPUT,
                             .pull_up_en = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    dataQueue = xQueueCreate(1, sizeof(uint32_t));
    if (dataQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create dataQueue");
        return;
    }

    xTaskCreate(slowTask, "SlowTask", 2048, NULL, 5, NULL);
    const esp_timer_create_args_t timer_args = {.callback = fastControlCallback, .arg = NULL, .dispatch_method = TIMER_DISPATCH_METHOD, .name = "FastControlTimer"};

    esp_timer_handle_t fastTimer;
    if (esp_timer_create(&timer_args, &fastTimer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create fastTimer");
        return;
    }

    if (esp_timer_start_periodic(fastTimer, 50) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start fastTimer");
        return;
    }
}
