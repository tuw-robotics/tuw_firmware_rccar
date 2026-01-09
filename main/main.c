#include "can/can_interface.h"
#include "mros/mros.h"
#include "odrive/odrive.h"
#include "odrive/odrive_enums.h"
#include "odrive/odrive_types.h"
#include "robot_controller/robot_controller.h"
#include "servo/servo.h"
#include "utils/indicator_led.h"
#include "utils/jtag_debug.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <sdkconfig.h>

#define MAIN_TAG "MAIN"

#define MOTOR_L_NODE_ID 0x00
#define MOTOR_L_CONTROL_MODE CONTROL_MODE_VELOCITY_CONTROL
#define MOTOR_L_INPUT_MODE INPUT_MODE_PASSTHROUGH
#define MOTOR_L_DIRECTION -1

#define MOTOR_R_NODE_ID 0x01
#define MOTOR_R_CONTROL_MODE CONTROL_MODE_VELOCITY_CONTROL
#define MOTOR_R_INPUT_MODE INPUT_MODE_PASSTHROUGH
#define MOTOR_R_DIRECTION 1

#ifdef CONFIG_SERVO_GPIO
#define SERVO_GPIO CONFIG_SERVO_GPIO
#else
#define SERVO_GPIO GPIO_NUM_5
#endif

#ifdef CONFIG_SERVO_CALIBRATED_MIDDLE_OFFSET
#define SERVO_CALIBRATED_MIDDLE_OFFSET CONFIG_SERVO_CALIBRATED_MIDDLE_OFFSET
#else
#define SERVO_CALIBRATED_MIDDLE_OFFSET 46
#endif

EventGroupHandle_t module_error_event_group = NULL;
#define ERROR_BIT_CAN BIT0
#define ERROR_BIT_MROS BIT1
#define ERROR_BIT_ROBOT_CONTROLLER BIT2

#define INIT_LED BIT0
#define INIT_CAN_MODULE BIT1
#define INIT_ODRIVE_ML_CONTEXT BIT2
#define INIT_ODRIVE_MR_CONTEXT BIT3
#define INIT_ODRIVES_STARTED BIT4
#define INIT_MROS_MODULE BIT5
#define INIT_MROS_STARTED BIT6
#define INIT_ROBOT_CONTROLLER BIT7
#define INIT_ROBOT_CONTROLLER_STARTED BIT8

static uint32_t init_status_flags = 0;
static EventBits_t module_status_bits = 0;

void app_main(void) {
    // Init indicator led
    if (led_indicator_init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize error indicator");
        goto error_handling;
    }
    init_status_flags |= INIT_LED;
    indicator_led_set_status(LED_STATUS_STARTUP);
    ESP_LOGI(MAIN_TAG, "Error indicator initialized successfully");

    // Init Global error handle event group
    module_error_event_group = xEventGroupCreate();
    if (module_error_event_group == NULL) {
        ESP_LOGE(MAIN_TAG, "Failed to create error handle event group");
        goto error_handling;
    }
    ESP_LOGI(MAIN_TAG, "Error handle event group created successfully");
    // Reset all error bits
    xEventGroupClearBits(module_error_event_group, ERROR_BIT_CAN | ERROR_BIT_MROS | ERROR_BIT_ROBOT_CONTROLLER);
    ESP_LOGI(MAIN_TAG, "Error handle event group created successfully");

    // Init jtag_debug
    if (jtag_debug_init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize JTAG debug");
        goto error_handling;
    }
    ESP_LOGI(MAIN_TAG, "JTAG debug initialized successfully");

    // Init can
    if (can_module_init(module_error_event_group, ERROR_BIT_CAN) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize CAN module");
        goto error_handling;
    }
    ESP_LOGI(MAIN_TAG, "CAN module initialized successfully");

    // Start can
    if (can_module_start() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to start CAN module");
        goto error_handling;
    }
    init_status_flags |= INIT_CAN_MODULE;
    ESP_LOGI(MAIN_TAG, "CAN module started successfully");

    // Init odrive contexts
    odrive_context_t odrive_ml_context;
    odrive_context_t odrive_mr_context;
    if (odrive_context_init(&odrive_ml_context, MOTOR_L_NODE_ID, MOTOR_L_CONTROL_MODE, MOTOR_L_INPUT_MODE, MOTOR_L_DIRECTION) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize ODrive ML context");
        goto error_handling;
    }
    init_status_flags |= INIT_ODRIVE_ML_CONTEXT;
    if (odrive_context_init(&odrive_mr_context, MOTOR_R_NODE_ID, MOTOR_R_CONTROL_MODE, MOTOR_R_INPUT_MODE, MOTOR_R_DIRECTION) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize ODrive MR context");
        goto error_handling;
    }
    init_status_flags |= INIT_ODRIVE_MR_CONTEXT;
    ESP_LOGI(MAIN_TAG, "ODrive contexts initialized successfully");
    // Start odrives
    if (odrive_startup(&odrive_ml_context) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to start ODrive ML");
        goto error_handling;
    }
    if (odrive_startup(&odrive_mr_context) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to start ODrive MR");
        goto error_handling;
    }
    init_status_flags |= INIT_ODRIVES_STARTED;
    ESP_LOGI(MAIN_TAG, "ODrive ML and MR started successfully");
    // Init mros
    if (mros_module_init(module_error_event_group, ERROR_BIT_MROS) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize MROS");
        goto error_handling;
    }
    init_status_flags |= INIT_MROS_MODULE;
    ESP_LOGI(MAIN_TAG, "MROS module initialized successfully");

    if (mros_module_start() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to start MROS");
        goto error_handling;
    }
    init_status_flags |= INIT_MROS_STARTED;
    ESP_LOGI(MAIN_TAG, "MROS module started successfully");
    // Init servo
    servo_t servo_context;

    if (servo_init(&servo_context, SERVO_GPIO, SERVO_CALIBRATED_MIDDLE_OFFSET) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize servo");
        goto error_handling;
    }
    ESP_LOGI(MAIN_TAG, "Servo initialized successfully");

    // Init robot_controller
    if (robot_controller_init(&odrive_ml_context, &odrive_mr_context, &servo_context, module_error_event_group, ERROR_BIT_ROBOT_CONTROLLER) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize robot controller");
        goto error_handling;
    }
    init_status_flags |= INIT_ROBOT_CONTROLLER;
    ESP_LOGI(MAIN_TAG, "Robot controller initialized successfully");

    // Start robot_controller
    if (robot_controller_start() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to start robot controller");
        goto error_handling;
    }
    init_status_flags |= INIT_ROBOT_CONTROLLER_STARTED;

    ESP_LOGI(MAIN_TAG, "Startup sequence completed successfully");

    indicator_led_set_status(LED_STATUS_OK);

    // Then wait for errors, if so, terminate and cleanup
    module_status_bits = xEventGroupWaitBits(module_error_event_group, ERROR_BIT_CAN | ERROR_BIT_MROS | ERROR_BIT_ROBOT_CONTROLLER, pdTRUE, pdFALSE, portMAX_DELAY);

    if (module_status_bits & ERROR_BIT_CAN) {
        ESP_LOGE(MAIN_TAG, "CAN error detected");
    }
    if (module_status_bits & ERROR_BIT_MROS) {
        ESP_LOGE(MAIN_TAG, "MROS error detected");
    }
    if (module_status_bits & ERROR_BIT_ROBOT_CONTROLLER) {
        ESP_LOGE(MAIN_TAG, "Robot controller error detected");
    }

error_handling:
    indicator_led_set_status(LED_STATUS_ERROR);

    ESP_LOGI(MAIN_TAG, "Handling error and trying to safely shutdown...");

    if ((init_status_flags & INIT_ODRIVES_STARTED) && !(module_status_bits & ERROR_BIT_CAN)) {
        ESP_LOGI(MAIN_TAG, "Setting ODrives to IDLE...");
        odrive_set_axis_state(&odrive_ml_context, AXIS_STATE_IDLE, true);
        odrive_set_axis_state(&odrive_mr_context, AXIS_STATE_IDLE, true);
    }

    if (init_status_flags & INIT_ROBOT_CONTROLLER_STARTED) {
        robot_controller_stop(pdMS_TO_TICKS(1000));
    }
    if (init_status_flags & INIT_ROBOT_CONTROLLER) {
        robot_controller_deinit(pdMS_TO_TICKS(1000));
    }

    if (init_status_flags & INIT_MROS_STARTED) {
        mros_module_stop(pdMS_TO_TICKS(1000));
    }
    if (init_status_flags & INIT_MROS_MODULE) {
        mros_module_deinit(pdMS_TO_TICKS(1000));
    }

    if (init_status_flags & INIT_CAN_MODULE) {
        can_module_stop(pdMS_TO_TICKS(1000));
        can_module_deinit(pdMS_TO_TICKS(1000));
    }

    if (init_status_flags & INIT_ODRIVE_ML_CONTEXT) {
        odrive_context_cleanup(&odrive_ml_context);
    }
    if (init_status_flags & INIT_ODRIVE_MR_CONTEXT) {
        odrive_context_cleanup(&odrive_mr_context);
    }

    // Deinit servo
    if (servo_deinit(&servo_context) != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to deinitialize servo");
    }

    // Deinit error handle event group
    if (module_error_event_group != NULL) {
        vEventGroupDelete(module_error_event_group);
        module_error_event_group = NULL;
    }
    ESP_LOGI(MAIN_TAG, "Deinitialization sequence completed successfully");

    //? Wait 5 seconds before restarting
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(MAIN_TAG, "Restarting...");
    esp_restart();
}