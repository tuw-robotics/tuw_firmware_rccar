#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "mros/mros_transport.h"
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <uros_network_interfaces.h>

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

#define TAG "PARAM"

#define RCCHECK(fn)                                                                                                                                                                                    \
    {                                                                                                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                                                                                                        \
        if ((temp_rc != RCL_RET_OK)) {                                                                                                                                                                 \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc);                                                                                                               \
            vTaskDelete(NULL);                                                                                                                                                                         \
        }                                                                                                                                                                                              \
    }
#define RCSOFTCHECK(fn)                                                                                                                                                                                \
    {                                                                                                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                                                                                                        \
        if ((temp_rc != RCL_RET_OK)) {                                                                                                                                                                 \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc);                                                                                                             \
        }                                                                                                                                                                                              \
    }

rclc_parameter_server_t param_server;

bool on_parameter_changed_cb(const rcl_interfaces__msg__Parameter *old_param, const rcl_interfaces__msg__Parameter *new_param, void *context) {
    (void)context;

    ESP_LOGI(MROS_LOGGER_TAG, "Parameter '%s' modified.", new_param->name.data);

    if (old_param) {
        ESP_LOGI(MROS_LOGGER_TAG, "Old value type: %d", (int)old_param->value.type);
        switch (old_param->value.type) {
        case RCLC_PARAMETER_INT:
            ESP_LOGI(MROS_LOGGER_TAG, "Old value: %lld (int64_t)", (long long)old_param->value.integer_value);
            break;
        case RCLC_PARAMETER_BOOL:
            ESP_LOGI(MROS_LOGGER_TAG, "Old value: %d (bool)", old_param->value.bool_value);
            break;
        case RCLC_PARAMETER_DOUBLE:
            ESP_LOGI(MROS_LOGGER_TAG, "Old value: %f (double)", old_param->value.double_value);
            break;
        default:
            ESP_LOGI(MROS_LOGGER_TAG, "Old value: unknown type");
            break;
        }
    } else {
        ESP_LOGI(MROS_LOGGER_TAG, "Old value is NULL (no previous value?)");
    }

    switch (new_param->value.type) {
    case RCLC_PARAMETER_BOOL:
        ESP_LOGI(MROS_LOGGER_TAG, "New value: %d (bool)", new_param->value.bool_value);
        break;
    case RCLC_PARAMETER_INT:
        ESP_LOGI(MROS_LOGGER_TAG, "New value: %lld (int64_t)", (long long)new_param->value.integer_value);
        break;
    case RCLC_PARAMETER_DOUBLE:
        ESP_LOGI(MROS_LOGGER_TAG, "New value: %f (double)", new_param->value.double_value);
        break;
    default:
        ESP_LOGW(MROS_LOGGER_TAG, "Unknown parameter type.");
        break;
    }

    return true;
}

void micro_ros_task(void *arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#if defined(RMW_TRANSPORT_UDP)
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    vTaskDelay(pdMS_TO_TICKS(100));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    vTaskDelay(pdMS_TO_TICKS(100));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_param_node", "", &support));

    rclc_parameter_options_t param_options = {
        .notify_changed_over_dds = false,
        .max_params = 1,
        .allow_undeclared_parameters = false,
        .low_mem_mode = false,
    };

    RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_options));

    vTaskDelay(pdMS_TO_TICKS(100));

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator));

    vTaskDelay(pdMS_TO_TICKS(100));
    RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed_cb));

    vTaskDelay(pdMS_TO_TICKS(100));
    RCCHECK(rclc_add_parameter(&param_server, "param1", RCLC_PARAMETER_INT));

    vTaskDelay(pdMS_TO_TICKS(100));
    int64_t param1_val = 10;
    RCCHECK(rclc_parameter_set_int(&param_server, "param1", param1_val));

    vTaskDelay(pdMS_TO_TICKS(100));
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        ESP_LOGI(MROS_LOGGER_TAG, "SPINNING");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_parameter_server_fini(&param_server, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

#define BUF_SIZE (1024)

void app_main(void) {
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    // To set the debug output to USB Serial JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    vTaskDelay(pdMS_TO_TICKS(100));

    rmw_uros_set_custom_transport(true, (void *)&uart_port, serial_open, serial_close, serial_write, serial_read);
#elif defined(RMW_UXRCE_TRANSPORT_UDP)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#else
#error micro-ROS transports misconfigured
#endif // RMW_UXRCE_TRANSPORT_CUSTOM

    vTaskDelay(pdMS_TO_TICKS(100));
    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
}