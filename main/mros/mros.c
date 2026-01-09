#include "mros.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <sys/time.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/config.h>

#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <uros_network_interfaces.h>

#include "mros_param.h"
#include "mros_transport.h"
#include "utils/timing_utils.h"

static rcl_allocator_t s_allocator;
static rclc_support_t s_support;
static rcl_node_t s_node;
static rclc_executor_t s_executor;
static rcl_timer_t s_odom_timer;
static rcl_publisher_t s_odom_publisher;
static rcl_subscription_t s_cmd_vel_subscriber;
static rclc_parameter_server_t s_param_server;

static TaskHandle_t s_mros_exec_task_h;
static TaskHandle_t s_mros_sync_task_h;

static EventGroupHandle_t s_mros_evt_group;
#define MROS_AGENT_CONNECTED BIT0
#define MROS_TIME_SYNCED BIT1
#define MROS_EXEC_RUN_BIT BIT2
#define MROS_EXEC_TERM_BIT BIT3
#define MROS_SYNC_RUN_BIT BIT4
#define MROS_SYNC_TERM_BIT BIT5

static EventGroupHandle_t s_mros_err_evt_group; // This is for external systems to be able to react to an error
static EventBits_t s_mros_err_bit;

#define EXECUTOR_HANDLES RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 2 // PARAM HANDLES + 2 for the timer and the subscription

static mros_cmd_vel_cb_t s_user_cmd_vel_cb;
static void *s_user_cmd_vel_ctx;

static geometry_msgs__msg__TwistStamped s_cmd_vel_buffer;
static QueueHandle_t s_odom_q;

static void internal_cmd_vel_callback(const void *msgin) {
    if (s_user_cmd_vel_cb) {
        s_user_cmd_vel_cb((const geometry_msgs__msg__TwistStamped *)msgin, s_user_cmd_vel_ctx);
    }
}

static void odom_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    ESP_UNUSED(timer);
    ESP_UNUSED(last_call_time);
    // TODO: For better real-time performance, it would probably be better to call publish in a non-mros timer with a higher priority
    //? This timer is only executed through the executor, so it can only be triggered as fast as the executor is spinned
    nav_msgs__msg__Odometry odom_msg;
    if (xQueuePeek(s_odom_q, &odom_msg, 0) != pdTRUE) { // Odom message is only peeked, not consumed -> so that publish rate is not affected
        ESP_LOGW(MROS_LOGGER_TAG, "No odometry data available in the queue");
        return;
    }

    if (rcl_publish(&s_odom_publisher, &odom_msg, NULL) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to publish odometry");
        return;
    }
}

static bool on_parameter_changed_callback(const rcl_interfaces__msg__Parameter *old_param, const rcl_interfaces__msg__Parameter *new_param, void *context) {
    ESP_UNUSED(old_param);
    ESP_UNUSED(context);

    return robot_parameters_handle_ros_change(new_param);
}

static void mros_executor_task(void *pv) {
    ESP_UNUSED(pv);
    ESP_LOGI(MROS_LOGGER_TAG, "Executor task started");

    // Wait untill sync has occured and start flag
    EventBits_t bits = xEventGroupWaitBits(s_mros_evt_group, MROS_AGENT_CONNECTED | MROS_TIME_SYNCED, pdFALSE, pdTRUE, MROS_MAX_TIMEOUT_MS);

    if (!(bits & MROS_AGENT_CONNECTED)) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to connect to micro-ROS agent");
        vTaskDelete(NULL);
    }
    if (!(bits & MROS_TIME_SYNCED)) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to sync time with micro-ROS agent");
        vTaskDelete(NULL);
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Agent connected and time synced");

    xEventGroupClearBits(s_mros_evt_group, MROS_EXEC_TERM_BIT);

    xEventGroupWaitBits(s_mros_evt_group, MROS_EXEC_RUN_BIT, pdFALSE, pdTRUE, portMAX_DELAY); // Wait for start signal

    while (true) {
        bits = xEventGroupGetBits(s_mros_evt_group);
        if (!(bits & MROS_EXEC_RUN_BIT)) {
            break;
        } else if (bits & MROS_AGENT_CONNECTED && bits & MROS_TIME_SYNCED) {
            // Executor handles timers, subscriptions and parameter server
            rclc_executor_spin_some(&s_executor, RCL_MS_TO_NS(MROS_EXECUTOR_SPIN_TIMEOUT_MS));
            taskYIELD(); // TODO: Check if we need this, maybe even a delay
        } else {
            ESP_LOGW(MROS_LOGGER_TAG, "Agent not connected or time not synced, waiting...");

            //? Should we terminate the task if we are not connected to the agent?
            xEventGroupWaitBits(s_mros_evt_group, MROS_AGENT_CONNECTED | MROS_TIME_SYNCED, pdFALSE, pdTRUE, portMAX_DELAY);
            // We could break the loop here, but we want to keep the task running to be able to reconnect
        }
    }

    if (bits & MROS_EXEC_RUN_BIT) { //? Currently there is no condition to break the loop, but if we add one, we need to check if the task was terminated or not
        xEventGroupSetBits(s_mros_err_evt_group, s_mros_err_bit);
        ESP_LOGE(MROS_LOGGER_TAG, "Executor task terminated unexpectedly");
    }

    xEventGroupSetBits(s_mros_evt_group, MROS_EXEC_TERM_BIT);
    ESP_LOGI(MROS_LOGGER_TAG, "Executor task stopping");
    s_mros_exec_task_h = NULL;
    vTaskDelete(NULL);
}

static esp_err_t mros_check_agent_connection(uint8_t attempts, monotonic_timestamp_t timeout) {
    if (rmw_uros_ping_agent(timeout, attempts) != RMW_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to ping agent");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t mros_sync_agent(wallclock_timestamp_t *timestamp, monotonic_timestamp_t timeout) {
    if (rmw_uros_sync_session(timeout) != RMW_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to synchronize time with agent");
        return ESP_FAIL;
    }

    if (!rmw_uros_epoch_synchronized()) {
        ESP_LOGE(MROS_LOGGER_TAG, "Epoch not synchronized");
        return ESP_FAIL;
    }

    int64_t epoch_nanos = rmw_uros_epoch_nanos();

    timestamp->tv_sec = NS_TO_S(epoch_nanos);
    timestamp->tv_usec = NS_SUBS_TO_USEC(epoch_nanos);

    return ESP_OK;
}

static void mros_sync_task(void *pv) {
    ESP_UNUSED(pv);
    ESP_LOGI(MROS_LOGGER_TAG, "Sync task started");
    xEventGroupClearBits(s_mros_evt_group, MROS_SYNC_TERM_BIT);

    EventBits_t bits = xEventGroupWaitBits(s_mros_evt_group, MROS_SYNC_RUN_BIT, pdFALSE, pdTRUE, MROS_MAX_TIMEOUT_MS);

    while (true) {

        bits = xEventGroupGetBits(s_mros_evt_group);
        if (!(bits & MROS_SYNC_RUN_BIT)) {
            break; // Instead of terminating every time the run bit is unset, we could just set it to a waiting state -> can be restarted again
        }

        // TODO: Reimplement this in a way that does not interfere with the executor task -> probably second mros executor?
        if (mros_check_agent_connection(MROS_MAX_PING_ATTEMPTS, MROS_PING_TIMEOUT_MS) == ESP_OK) {
            xEventGroupSetBits(s_mros_evt_group, MROS_AGENT_CONNECTED);
        } else {
            xEventGroupClearBits(s_mros_evt_group, MROS_AGENT_CONNECTED);
            ESP_LOGW(MROS_LOGGER_TAG, "Failed to ping micro-ROS agent");
        }

        if (bits & MROS_AGENT_CONNECTED) {
            wallclock_timestamp_t wts;
            if (mros_sync_agent(&wts, MROS_MAX_TIME_SYNC_TIMEOUT_MS) == ESP_OK) {
                if (settimeofday(&wts, NULL) != 0) { // TODO: Set timezone
                    ESP_LOGE(MROS_LOGGER_TAG, "Failed to set time on system");
                    xEventGroupClearBits(s_mros_evt_group, MROS_TIME_SYNCED);
                } else {
                    xEventGroupSetBits(s_mros_evt_group, MROS_TIME_SYNCED);
                    //! We now exit -> should be handled cleaner
                    ESP_LOGI(MROS_LOGGER_TAG, "Time synchronized with agent");
                    ESP_LOGI(MROS_LOGGER_TAG, "Current time: %lld.%06ld", wts.tv_sec, wts.tv_usec);
                    ESP_LOGI(MROS_LOGGER_TAG, "Time sync task exiting");
                    xEventGroupClearBits(s_mros_evt_group, MROS_SYNC_RUN_BIT);
                }
            } else {
                xEventGroupClearBits(s_mros_evt_group, MROS_TIME_SYNCED);
                ESP_LOGW(MROS_LOGGER_TAG, "Failed to sync time with agent");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MROS_AGENT_PERIOD_MS));
    }

    if (bits & MROS_SYNC_RUN_BIT) { //? Currently there is no condition to break the loop, but if we add one, we need to check if the task was terminated or not
        xEventGroupSetBits(s_mros_err_evt_group, s_mros_err_bit);
        ESP_LOGE(MROS_LOGGER_TAG, "Sync task terminated unexpectedly");
    }

    xEventGroupSetBits(s_mros_evt_group, MROS_SYNC_TERM_BIT);
    ESP_LOGI(MROS_LOGGER_TAG, "Sync task stopped");
    s_mros_sync_task_h = NULL;
    vTaskDelete(NULL);
}

esp_err_t mros_module_init(EventGroupHandle_t error_handle, EventBits_t error_bit) {
    if (!error_handle) {
        ESP_LOGE(MROS_LOGGER_TAG, "Invalid error handle");
        return ESP_ERR_INVALID_ARG;
    }

    s_mros_err_evt_group = error_handle;
    s_mros_err_bit = error_bit;
    xEventGroupClearBits(s_mros_err_evt_group, s_mros_err_bit);

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM) // TODO: Make this cleaner configurable
    static size_t uart_port = MROS_SERIAL_PORT;
    if (rmw_uros_set_custom_transport(true, (void *)&uart_port, serial_open, serial_close, serial_write, serial_read) != RMW_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set custom transport");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Serial transport set");
#else
    if (uros_network_interface_initialize() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize network interface");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Network interface initialized");
#endif

    s_odom_q = xQueueCreate(1, sizeof(nav_msgs__msg__Odometry));
    if (s_odom_q == NULL) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to create odom queue");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Odom queue created");

    s_allocator = rcl_get_default_allocator();
    if (rclc_support_init(&s_support, 0, NULL, &s_allocator) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize rcl support");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "rcl support initialized");

    s_node = rcl_get_zero_initialized_node();
    if (rclc_node_init_default(&s_node, MROS_NODE_NAME, MROS_NODE_NAMESPACE, &s_support) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize node");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Node initialized");

    // TODO best effort should be used
    //! Why doesnt best effort work? Works in mros_test example...
    if (rclc_publisher_init_best_effort(&s_odom_publisher, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), MROS_ODOMETRY_TOPIC) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize odom publisher");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Odometry publisher initialized");

    if (rclc_timer_init_default2(&s_odom_timer, &s_support, RCL_MS_TO_NS(MROS_ODOM_PUBLISHER_PERIOD_MS), odom_timer_callback, true) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize timer");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Timer initialized");

    if (rclc_parameter_server_init_default(&s_param_server, &s_node) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize parameter server");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Parameter server initialized");

    if (robot_parameters_init() != ESP_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize persistent parameters");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Persistent parameters initialized");

    if (robot_parameters_register_all(&s_param_server) != ESP_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add robot parameters");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Robot parameters registered to micro-ROS");

    //! When subscribing to a reliable topic that has a high publishing frequency, the acknowledgements might block other things in the executer
    if (rclc_subscription_init_default(&s_cmd_vel_subscriber, &s_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), MROS_CMD_VEL_TOPIC) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize cmd_vel subscriber");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "cmd_vel subscriber initialized");

    s_executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&s_executor, &s_support.context, EXECUTOR_HANDLES, &s_allocator) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to initialize executor");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Executor initialized");

    if (rclc_executor_add_subscription(&s_executor, &s_cmd_vel_subscriber, &s_cmd_vel_buffer, internal_cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add cmd_vel subscription");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "cmd_vel subscription added to executor");

    if (rclc_executor_add_parameter_server(&s_executor, &s_param_server, on_parameter_changed_callback) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add parameter server to executor");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Parameter server added to executor");

    if (rclc_executor_add_timer(&s_executor, &s_odom_timer) != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add timer to executor");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Timer added to executor");

    s_mros_evt_group = xEventGroupCreate();
    if (!s_mros_evt_group) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to create event group");
        return ESP_FAIL;
    }
    xEventGroupSetBits(s_mros_evt_group, MROS_EXEC_TERM_BIT | MROS_SYNC_TERM_BIT);
    xEventGroupClearBits(s_mros_evt_group, MROS_EXEC_RUN_BIT | MROS_SYNC_RUN_BIT | MROS_AGENT_CONNECTED | MROS_TIME_SYNCED);
    ESP_LOGI(MROS_LOGGER_TAG, "Event group created");

    // Probably not needed, but so that we have a clearly defined state
    s_mros_sync_task_h = NULL;
    s_mros_exec_task_h = NULL;
    s_user_cmd_vel_cb = NULL;
    s_user_cmd_vel_ctx = NULL;

    ESP_LOGI(MROS_LOGGER_TAG, "Module initialized successfully");
    return ESP_OK;
}

esp_err_t mros_module_start(void) {
    if (!s_mros_exec_task_h) {
        if (xTaskCreate(mros_executor_task, MROS_EXECUTOR_TASK_NAME, MROS_EXECUTOR_TASK_STACK_SIZE, NULL, MROS_EXECUTOR_TASK_PRIORITY, &s_mros_exec_task_h) != pdPASS) {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to create executor task");
            s_mros_exec_task_h = NULL;
            return ESP_FAIL;
        }
    }

    if (!s_mros_sync_task_h) {
        if (xTaskCreate(mros_sync_task, MROS_AGENT_TASK_NAME, MROS_AGENT_TASK_STACK_SIZE, NULL, MROS_AGENT_TASK_PRIORITY, &s_mros_sync_task_h) != pdPASS) {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to create agent task");
            // Force termination of executor task //! not a nice way to do this
            vTaskDelete(s_mros_exec_task_h); // Not sure if this is the best way, as at this point, the task is waiting for sync to be finished
            xEventGroupSetBits(s_mros_evt_group, MROS_EXEC_TERM_BIT);
            s_mros_exec_task_h = NULL;
            s_mros_sync_task_h = NULL;
            return ESP_FAIL;
        }
    }

    xEventGroupSetBits(s_mros_evt_group, MROS_EXEC_RUN_BIT | MROS_SYNC_RUN_BIT);

    ESP_LOGI(MROS_LOGGER_TAG, "Module started");
    return ESP_OK;
}

esp_err_t mros_module_stop(TickType_t wait_ticks) {
    if (!s_mros_exec_task_h && !s_mros_sync_task_h) {
        ESP_LOGI(MROS_LOGGER_TAG, "Module not running");
        return ESP_OK; // Already stopped
    }

    ESP_LOGI(MROS_LOGGER_TAG, "Stopping module...");
    xEventGroupClearBits(s_mros_evt_group, MROS_EXEC_RUN_BIT | MROS_SYNC_RUN_BIT);

    EventBits_t bits = xEventGroupWaitBits(s_mros_evt_group, MROS_EXEC_TERM_BIT | MROS_SYNC_TERM_BIT, pdFALSE, pdTRUE, wait_ticks);
    if (!(bits & MROS_SYNC_TERM_BIT)) {
        ESP_LOGE(MROS_LOGGER_TAG, "Tasks did not stop in time");
        return ESP_FAIL;
    }
    ESP_LOGI(MROS_LOGGER_TAG, "Module stopped");
    return ESP_OK;
}

esp_err_t mros_module_deinit(TickType_t wait_ticks) {
    if (mros_module_stop(wait_ticks) != ESP_OK) {
        return ESP_FAIL;
    }
    rclc_executor_fini(&s_executor);
    rclc_parameter_server_fini(&s_param_server, &s_node);

    if (rcl_publisher_fini(&s_odom_publisher, &s_node) != RCL_RET_OK) {
        ESP_LOGW(MROS_LOGGER_TAG, "Failed to finalize odom publisher");
    }
    if (rcl_subscription_fini(&s_cmd_vel_subscriber, &s_node) != RCL_RET_OK) {
        ESP_LOGW(MROS_LOGGER_TAG, "Failed to finalize cmd_vel subscriber");
    }
    if (rcl_timer_fini(&s_odom_timer) != RCL_RET_OK) {
        ESP_LOGW(MROS_LOGGER_TAG, "Failed to finalize odom timer");
    }
    if (rcl_node_fini(&s_node) != RCL_RET_OK) {
        ESP_LOGW(MROS_LOGGER_TAG, "Failed to finalize node");
    }
    rclc_support_fini(&s_support);

    if (s_mros_evt_group) {
        vEventGroupDelete(s_mros_evt_group);
        s_mros_evt_group = NULL;
    }

    if (s_odom_q) {
        vQueueDelete(s_odom_q);
        s_odom_q = NULL;
    }

    if (s_mros_exec_task_h) {
        vTaskDelete(s_mros_exec_task_h);
        s_mros_exec_task_h = NULL;
    }

    if (s_mros_sync_task_h) {
        vTaskDelete(s_mros_sync_task_h);
        s_mros_sync_task_h = NULL;
    }

    s_odom_q = NULL;
    s_mros_exec_task_h = NULL;
    s_mros_sync_task_h = NULL;
    s_user_cmd_vel_cb = NULL;
    s_user_cmd_vel_ctx = NULL;
    s_mros_evt_group = NULL;

    s_mros_err_evt_group = NULL;

    ESP_LOGI(MROS_LOGGER_TAG, "Module deinitialized");
    return ESP_OK;
}

esp_err_t mros_register_cmd_vel_callback(mros_cmd_vel_cb_t callback, void *context) {
    s_user_cmd_vel_cb = callback;
    s_user_cmd_vel_ctx = context;
    return ESP_OK;
}

esp_err_t mros_update_odometry(nav_msgs__msg__Odometry *odom_msg) {
    if (xQueueOverwrite(s_odom_q, odom_msg) != pdTRUE) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to overwrite odom queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mros_init_odometry_msg(nav_msgs__msg__Odometry *odom_msg) {
    if (odom_msg == NULL) {
        ESP_LOGE(MROS_LOGGER_TAG, "Invalid argument to init_odom_msg");
        return ESP_ERR_INVALID_ARG;
    }

    memset(odom_msg, 0, sizeof(nav_msgs__msg__Odometry));

    rosidl_runtime_c__String__assign(&odom_msg->header.frame_id, MROS_ODOM_FRAME_ID);
    rosidl_runtime_c__String__assign(&odom_msg->child_frame_id, MROS_BASE_FRAME_ID);

    //! The covariance values are currently only placeholders
    // TODO: Set the covariance values to something meaningful -> possibly also configure them through parameters
    odom_msg->pose.covariance[0] = 0.01;  // x
    odom_msg->pose.covariance[7] = 0.01;  // y
    odom_msg->pose.covariance[14] = 1e9;  // z
    odom_msg->pose.covariance[21] = 1e9;  // roll
    odom_msg->pose.covariance[28] = 1e9;  // pitch
    odom_msg->pose.covariance[35] = 0.01; // yaw

    odom_msg->twist.covariance[0] = 0.005;  // vx
    odom_msg->twist.covariance[7] = 1e9;    // vy
    odom_msg->twist.covariance[14] = 1e9;   // vz
    odom_msg->twist.covariance[21] = 1e9;   // wx
    odom_msg->twist.covariance[28] = 1e9;   // wy
    odom_msg->twist.covariance[35] = 0.005; // wz

    odom_msg->twist.twist.linear.y = 0.0f;
    odom_msg->twist.twist.linear.z = 0.0f;
    odom_msg->twist.twist.angular.x = 0.0f;
    odom_msg->twist.twist.angular.y = 0.0f;

    return ESP_OK;
}

bool mros_is_agent_connected(void) { return xEventGroupGetBits(s_mros_evt_group) & MROS_AGENT_CONNECTED; }

bool mros_is_time_synced(void) { return xEventGroupGetBits(s_mros_evt_group) & MROS_TIME_SYNCED; }