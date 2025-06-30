#include "driver/gpio.h"
#include "driver/uart.h"
#include "utils/timing_utils.h"

#include "driver/usb_serial_jtag.h"
#include "mros/mros.h"
#include "odrive/odrive_types.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <sys/time.h>
#include <time.h>

#define ERROR_LED_GPIO GPIO_NUM_21

void micro_ros_task(void *arg) {

    ESP_ERROR_CHECK(mros_init());

    if (check_agent_connection(3, S_TO_MS(1)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to micro-ROS agent");
        gpio_set_level(ERROR_LED_GPIO, 1);
        vTaskDelete(NULL);
    }

    wallclock_timestamp_t wall_time;
    if (sync_agent(&wall_time, S_TO_MS(10)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to sync with agent");
        gpio_set_level(ERROR_LED_GPIO, 1);
        vTaskDelete(NULL);
    }

    if (settimeofday(&wall_time, NULL) != 0) {
        ESP_LOGE(TAG, "Failed to set time");
        gpio_set_level(ERROR_LED_GPIO, 1);
        vTaskDelete(NULL);
    }

    while (true) {

        if (check_agent_connection(3, S_TO_MS(1)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to connect to micro-ROS agent");
            gpio_set_level(ERROR_LED_GPIO, 1);
            vTaskDelete(NULL);
        }
        geometry_msgs__msg__TwistStamped cmd_vel_msg;
        if (xQueueReceive(cmd_vel_queue, &cmd_vel_msg, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Received cmd_vel message: linear.x=%f, angular.z=%f", cmd_vel_msg.twist.linear.x, cmd_vel_msg.twist.angular.z);
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // free resources
    ESP_ERROR_CHECK(mros_cleanup());
    ESP_LOGI(TAG, "Micro-ROS task stopped");
    gpio_set_level(ERROR_LED_GPIO, 1);

    vTaskDelete(NULL);
}

void odom_generator(void *arg) {

    while (true) {
        nav_msgs__msg__Odometry odom_msg;
        wallclock_timestamp_t wall_time;

        gettimeofday(&wall_time, NULL);

        odom_msg.header.stamp.sec = wall_time.tv_sec;
        odom_msg.header.stamp.nanosec = wall_time.tv_usec * 1000;
        odom_msg.header.frame_id.data = "odom";
        odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
        odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
        odom_msg.child_frame_id.data = "base_link";
        odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
        odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;
        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;
        for (int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
        }

        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        for (int i = 0; i < 36; i++) {
            odom_msg.twist.covariance[i] = 0.0;
        }

        // Now add the odom_msg to the queue
        if (xQueueOverwrite(odom_queue, &odom_msg) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to overwrite odom queue");
            gpio_set_level(ERROR_LED_GPIO, 1);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

#define BUF_SIZE (1024)

void app_main(void) {
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI(TAG, "USB_SERIAL_JTAG init done");

    gpio_reset_pin(ERROR_LED_GPIO);
    gpio_set_direction(ERROR_LED_GPIO, GPIO_MODE_OUTPUT);

    xTaskCreate(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    xTaskCreate(odom_generator, "odom_gen_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    ESP_LOGI(TAG, "Tasks created");
}
