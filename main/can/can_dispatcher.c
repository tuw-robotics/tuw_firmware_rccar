#include "can_dispatcher.h"
#include "can_driver.h"
#include "can_rtr.h"
#include "utils/timing_utils.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <sys/time.h>

static disp_entry_t s_entries[MAX_CBS];
static size_t s_num_entries = 0;
static TaskHandle_t s_task_h = NULL;

static EventGroupHandle_t s_evt_group = NULL;
#define CAN_DISP_RUN_BIT BIT0
#define CAN_DISP_TERM_BIT BIT1

static EventGroupHandle_t s_can_err_evt_group; // This is for external systems to be able to react to an error
static EventBits_t s_can_err_bit;

static void dispatcher_task(void *pv) {
    ESP_UNUSED(pv);
    ESP_LOGI(CAN_DISPATCHER_TAG, "Dispatcher task started");
    xEventGroupClearBits(s_evt_group, CAN_DISP_TERM_BIT);

    twai_message_t rx_msg;

    EventBits_t bits = xEventGroupWaitBits(s_evt_group, CAN_DISP_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // Clear the receive queue on first execution.
    if (can_clear_receive_queue() != ESP_OK) {
        ESP_LOGW(CAN_DISPATCHER_TAG, "Failed to clear TWAI receive queue on start");
    } else {
        ESP_LOGI(CAN_DISPATCHER_TAG, "TWAI receive queue cleared");
    }

    while (true) {
        esp_err_t receive_ret = can_driver_receive(&rx_msg, pdMS_TO_TICKS(CAN_MAX_RECEIVE_TIMEOUT_MS)); // Blocking wait for message

        if (!(xEventGroupGetBits(s_evt_group) & CAN_DISP_RUN_BIT)) {
            break;
        }

        if (receive_ret == ESP_OK) {
            monotonic_timestamp_t monotonic_time = esp_timer_get_time();
            wallclock_timestamp_t wall_time;
            gettimeofday(&wall_time, NULL);

            if (can_rtr_handle_incoming(&rx_msg, wall_time, monotonic_time)) { // Check if it's an RTR response
                continue;
            }

            // Not an RTR response, dispatch to registered callbacks
            bool dispatched = false;
            for (size_t i = 0; i < s_num_entries; ++i) {
                if (s_entries[i].msg_id == rx_msg.identifier) {
                    s_entries[i].cb(rx_msg.identifier, rx_msg.data, rx_msg.data_length_code, wall_time, monotonic_time, s_entries[i].ctx);
                    dispatched = true;
                }
            }
            if (!dispatched) {
                ESP_LOGD(CAN_DISPATCHER_TAG, "Received unhandled CAN message: ID=0x%03lX", rx_msg.identifier);
            }

        } else if (receive_ret == ESP_ERR_TIMEOUT) {
            // Timeout, continue waiting for messages
            continue;
        } else {
            // TODO: Implement recovery, for now just log the error and break
            ESP_LOGE(CAN_DISPATCHER_TAG, "Failed to receive CAN message: %s", esp_err_to_name(receive_ret));
            break;
        }
    }

    if ((bits & CAN_DISP_RUN_BIT)) {
        xEventGroupSetBits(s_can_err_evt_group, s_can_err_bit);
        ESP_LOGE(CAN_DISPATCHER_TAG, "Dispatcher task terminated unexpectedly");
    }

    ESP_LOGI(CAN_DISPATCHER_TAG, "Dispatcher task exiting");
    xEventGroupSetBits(s_evt_group, CAN_DISP_TERM_BIT);
    s_task_h = NULL;
    vTaskDelete(NULL);
}

esp_err_t can_dispatcher_init(EventGroupHandle_t error_handle, EventBits_t error_bit) {
    if (!error_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    s_can_err_evt_group = error_handle;
    s_can_err_bit = error_bit;
    xEventGroupClearBits(s_can_err_evt_group, s_can_err_bit);

    s_num_entries = 0;
    s_task_h = NULL;
    if (s_evt_group == NULL) {
        s_evt_group = xEventGroupCreate();
        if (s_evt_group == NULL) {
            ESP_LOGE(CAN_DISPATCHER_TAG, "Failed to create event group");
            return ESP_FAIL;
        }
    }
    xEventGroupSetBits(s_evt_group, CAN_DISP_TERM_BIT); // To indicate task is not running
    xEventGroupClearBits(s_evt_group, CAN_DISP_RUN_BIT | CAN_DISP_TERM_BIT);
    return ESP_OK;
}

esp_err_t can_dispatcher_register(uint32_t msg_id, can_message_callback_t cb, void *ctx) {
    if (s_num_entries >= MAX_CBS) {
        ESP_LOGE(CAN_DISPATCHER_TAG, "Maximum number of callbacks (%d) reached", MAX_CBS);
        return ESP_FAIL;
    }

    s_entries[s_num_entries].msg_id = msg_id;
    s_entries[s_num_entries].cb = cb;
    s_entries[s_num_entries].ctx = ctx;
    s_num_entries++;

    ESP_LOGI(CAN_DISPATCHER_TAG, "Registered handler for ID=0x%03lX (%d/%d used)", msg_id, s_num_entries, MAX_CBS);
    return ESP_OK;
}

esp_err_t can_dispatcher_unregister(uint32_t msg_id, can_message_callback_t cb, void *ctx) {
    for (size_t i = 0; i < s_num_entries; ++i) {
        if (s_entries[i].msg_id == msg_id && s_entries[i].cb == cb && s_entries[i].ctx == ctx) {
            // Shift remaining entries down
            for (size_t j = i; j < s_num_entries - 1; ++j) { //? Linked list would be cleaner
                s_entries[j] = s_entries[j + 1];
            }
            s_num_entries--;
            ESP_LOGI(CAN_DISPATCHER_TAG, "Unregistered handler for ID=0x%03lX (%d/%d used)", msg_id, s_num_entries, MAX_CBS);
            return ESP_OK;
        }
    }
    // ESP_LOGW(CAN_DISPATCHER_TAG, "Handler for ID=0x%03lX not found", msg_id);
    return ESP_FAIL;
}

esp_err_t can_dispatcher_start(void) {
    if (s_task_h != NULL) {
        ESP_LOGW(CAN_DISPATCHER_TAG, "Dispatcher task already started");
        return ESP_OK; // Already running
    }

    if (!can_driver_is_running()) {
        if (can_driver_start() != ESP_OK) {
            ESP_LOGE(CAN_DISPATCHER_TAG, "Failed to start CAN driver");
            return ESP_FAIL;
        }
    }

    if (xTaskCreate(dispatcher_task, TASK_NAME, TASK_STACK_SZ, NULL, TASK_PRIO, &s_task_h) != pdPASS) {
        ESP_LOGE(CAN_DISPATCHER_TAG, "Failed to create dispatcher task");
        s_task_h = NULL;
        return ESP_FAIL;
    }

    xEventGroupSetBits(s_evt_group, CAN_DISP_RUN_BIT);
    return ESP_OK;
}

esp_err_t can_dispatcher_stop(TickType_t wait_ticks) {
    if (s_task_h == NULL) {
        ESP_LOGI(CAN_DISPATCHER_TAG, "Dispatcher task not running");
        return ESP_OK; // Already stopped
    }

    ESP_LOGI(CAN_DISPATCHER_TAG, "Stopping dispatcher task...");
    xEventGroupClearBits(s_evt_group, CAN_DISP_RUN_BIT);

    // Stop the CAN driver so that can_driver_receive terminates
    if (can_driver_stop() != ESP_OK) {
        ESP_LOGE(CAN_DISPATCHER_TAG, "Failed to send stop signal to CAN driver, dispatcher task might not terminate cleanly.");
    }

    // Wait for the task to terminate within timeout
    EventBits_t bits = xEventGroupWaitBits(s_evt_group, CAN_DISP_TERM_BIT, pdTRUE, pdFALSE, wait_ticks);
    if ((bits & CAN_DISP_TERM_BIT) == 0) {
        ESP_LOGE(CAN_DISPATCHER_TAG, "Dispatcher task did not terminate in time");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t can_dispatcher_deinit(TickType_t wait_ticks) {
    if (can_dispatcher_stop(wait_ticks) != ESP_OK) {
        return ESP_FAIL;
    }

    if (s_evt_group) {
        vEventGroupDelete(s_evt_group);
        s_evt_group = NULL;
    }

    s_num_entries = 0;
    s_task_h = NULL;

    s_can_err_evt_group = NULL;

    return ESP_OK;
}