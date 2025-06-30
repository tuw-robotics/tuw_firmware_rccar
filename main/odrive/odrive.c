#include "odrive/odrive.h"
#include "can/can_interface.h"
#include "odrive/odrive_can.h"
#include "odrive/odrive_enums.h"
#include "odrive/odrive_types.h"
#include "utils/timing_utils.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

// TODO: We should probably check if CAN is initialized and started before doing anything

static void odrive_heartbeat_callback(uint32_t msg_id, const uint8_t *data, size_t dlc, wallclock_timestamp_t wall_time, monotonic_timestamp_t monotonic_time, void *context) {
    odrive_context_t *odrive_context = (odrive_context_t *)context;

    odrive_stamped_payload_t heartbeat_stamped;
    memcpy(&heartbeat_stamped.payload.heartbeat, data, sizeof(odrive_heartbeat_t));
    heartbeat_stamped.wall_time = wall_time;
    heartbeat_stamped.monotonic_time = monotonic_time;

    if (xQueueOverwrite(odrive_context->heartbeat_queue, &heartbeat_stamped) != pdPASS) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to overwrite heartbeat queue for Node ID %d", odrive_context->node_id);
    }
}

static void odrive_encoder_estimates_callback(uint32_t msg_id, const uint8_t *data, size_t dlc, wallclock_timestamp_t wall_time, monotonic_timestamp_t monotonic_time, void *context) {
    odrive_context_t *odrive_context = (odrive_context_t *)context;

    odrive_stamped_payload_t estimates_stamped;
    memcpy(&estimates_stamped.payload.encoder_estimates, data, sizeof(odrive_encoder_estimates_t));
    estimates_stamped.wall_time = wall_time;
    estimates_stamped.monotonic_time = monotonic_time;

    if (xQueueOverwrite(odrive_context->encoder_estimates_queue, &estimates_stamped) != pdPASS) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to overwrite encoder estimates queue for Node ID %d", odrive_context->node_id);
    }

    // TODO: Check for ODRIVE_ENCODER_ESTIMATES_TIMEOUT_MS
}

esp_err_t odrive_context_init(odrive_context_t *context, uint8_t node_id, ControlMode control_mode, InputMode input_mode, int8_t direction) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    memset(context, 0, sizeof(odrive_context_t)); // Apparently best practise
    context->node_id = node_id;
    context->control_mode = control_mode;
    context->input_mode = input_mode;
    context->direction = direction;

    context->heartbeat_queue = xQueueCreate(1, sizeof(odrive_stamped_payload_t)); // We only care about the latest heartbeat -> size 1
    if (context->heartbeat_queue == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to create heartbeat queue for Node ID %d", node_id);
        goto cleanup; // Gotos are apparently not considered bad practice in this case
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Heartbeat queue created for Node ID %d", node_id);

    context->encoder_estimates_queue = xQueueCreate(1, sizeof(odrive_stamped_payload_t));
    if (context->encoder_estimates_queue == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to create encoder estimates queue for Node ID %d", node_id);
        goto cleanup;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Encoder estimates queue created for Node ID %d", node_id);

    uint32_t heartbeat_msg_id = odrive_can_pack_id(context->node_id, CMD_HEARTBEAT);
    if (can_register_message_handler(heartbeat_msg_id, odrive_heartbeat_callback, context) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to register heartbeat handler (ID: 0x%03lX) for Node ID %d", heartbeat_msg_id, node_id);
        goto cleanup;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Registered heartbeat handler (ID: 0x%03lX) for Node ID %d", heartbeat_msg_id, node_id);

    uint32_t encoder_msg_id = odrive_can_pack_id(context->node_id, CMD_GET_ENCODER_ESTIMATES);
    if (can_register_message_handler(encoder_msg_id, odrive_encoder_estimates_callback, context) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to register encoder estimates handler (ID: 0x%03lX) for Node ID %d", encoder_msg_id, node_id);
        goto cleanup;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Registered encoder estimates handler (ID: 0x%03lX) for Node ID %d", encoder_msg_id, node_id);

    return ESP_OK;

cleanup:
    odrive_context_cleanup(context);
    return ESP_FAIL;
}

esp_err_t odrive_context_cleanup(odrive_context_t *context) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t heartbeat_msg_id = odrive_can_pack_id(context->node_id, CMD_HEARTBEAT);
    if (can_unregister_message_handler(heartbeat_msg_id, odrive_heartbeat_callback, context) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to unregister heartbeat handler (ID: 0x%03lX) for Node ID %d", heartbeat_msg_id, context->node_id);
    } else {
        ESP_LOGI(ODRIVE_LOGGER_TAG, "Unregistered heartbeat handler (ID: 0x%03lX) for Node ID %d", heartbeat_msg_id, context->node_id);
    }
    uint32_t encoder_msg_id = odrive_can_pack_id(context->node_id, CMD_GET_ENCODER_ESTIMATES);
    if (can_unregister_message_handler(encoder_msg_id, odrive_encoder_estimates_callback, context) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to unregister encoder estimates handler (ID: 0x%03lX) for Node ID %d", encoder_msg_id, context->node_id);
    } else {
        ESP_LOGI(ODRIVE_LOGGER_TAG, "Unregistered encoder estimates handler (ID: 0x%03lX) for Node ID %d", encoder_msg_id, context->node_id);
    }

    if (context->heartbeat_queue != NULL) {
        vQueueDelete(context->heartbeat_queue);
        context->heartbeat_queue = NULL;
        ESP_LOGI(ODRIVE_LOGGER_TAG, "Heartbeat queue deleted for Node ID %d", context->node_id);
    }

    if (context->encoder_estimates_queue != NULL) {
        vQueueDelete(context->encoder_estimates_queue);
        context->encoder_estimates_queue = NULL;
        ESP_LOGI(ODRIVE_LOGGER_TAG, "Encoder estimates queue deleted for Node ID %d", context->node_id);
    }

    return ESP_OK;
}

esp_err_t odrive_prepare_initial_state(odrive_context_t *context) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    AxisState axis_state;
    if (odrive_get_axis_state(context, &axis_state) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to get initial ODrive axis state for Node ID %d", context->node_id);
        return ESP_FAIL;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Initial axis state of ODrive Node ID %d: %d", context->node_id, axis_state);

    // If device is in an unexpected state (probably in some calibration state), wait until it becomes idle
    // TODO: Handle this better and actually check the state
    if (axis_state != AXIS_STATE_IDLE && axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        ESP_LOGI(ODRIVE_LOGGER_TAG, "ODrive Node ID %d not in idle or closed-loop. Waiting for it to become idle..", context->node_id);
        if (odrive_wait_state(context, AXIS_STATE_IDLE, pdMS_TO_TICKS(S_TO_MS(ODRIVE_MAX_CALIBRATION_TIME_S))) != ESP_OK) {
            ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed waiting for ODrive Node ID %d to enter idle state", context->node_id);
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

esp_err_t odrive_startup(odrive_context_t *context) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Discover
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Discovering ODrive Node ID %d...", context->node_id);
    if (odrive_discover(context) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to discover ODrive Node ID %d", context->node_id);
        return ESP_FAIL;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "ODrive Node ID %d discovered", context->node_id);

    // 2. Check Heartbeat reception
    odrive_stamped_payload_t heartbeat;
    if (xQueuePeek(context->heartbeat_queue, &heartbeat, pdMS_TO_TICKS(ODRIVE_HEARTBEAT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "No heartbeat received from ODrive Node ID %d after discovery", context->node_id);
        return ESP_FAIL;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Initial heartbeat received from Node ID %d", context->node_id);

    // 3. Prepare Initial State (Wait for IDLE if necessary)
    if (odrive_prepare_initial_state(context) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to prepare initial state for ODrive Node ID %d", context->node_id);
        return ESP_FAIL;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "ODrive Node ID %d is ready for configuration", context->node_id);

    // 4. Set Control Mode
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Setting ODrive Node ID %d control mode to %d", context->node_id, context->control_mode);
    if (odrive_set_control_mode(context, context->control_mode, context->input_mode) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to set ODrive Node ID %d control mode", context->node_id);
        return ESP_FAIL;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "ODrive Node ID %d control mode set", context->node_id);
    //! No confirmation possible via standard CAN messages -> we have to assume it worked

    // 5. Set Axis State to Closed Loop
    ESP_LOGI(ODRIVE_LOGGER_TAG, "Setting ODrive Node ID %d axis state to CLOSED_LOOP (%d)", context->node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
    if (odrive_set_axis_state(context, AXIS_STATE_CLOSED_LOOP_CONTROL, true) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to set ODrive Node ID %d axis state to closed loop", context->node_id);
        return ESP_FAIL;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG, "ODrive Node ID %d axis state set to closed loop", context->node_id);

    ESP_LOGI(ODRIVE_LOGGER_TAG, "ODrive Node ID %d startup sequence complete", context->node_id);
    return ESP_OK;
}

esp_err_t odrive_discover(odrive_context_t *context) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t msg_id = odrive_can_pack_id(context->node_id, CMD_ADRESS);
    if (can_send_rtr_request(msg_id, ODRIVE_CAN_DATA_LENGTH, MS_TO_US(ODRIVE_CAN_MAX_TRANSMIT_QUEUE_TIMEOUT_MS), MS_TO_US(ODRIVE_CAN_MAX_RTR_RESPONSE_TIME_MS)) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to send ODrive discovery RTR request for Node ID %d (ID: 0x%03lX)", context->node_id, msg_id);
        return ESP_FAIL;
    }

    ESP_LOGD(ODRIVE_LOGGER_TAG, "Waiting for RTR response for ID 0x%03lX", msg_id);
    odrive_address_t address_response;
    if (can_receive_rtr_data(msg_id, &address_response, sizeof(address_response), pdMS_TO_TICKS(ODRIVE_CAN_MAX_RECEIVE_QUEUE_TIMEOUT_MS)) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to receive ODrive discovery response for Node ID %d (ID: 0x%03lX)", context->node_id, msg_id);
        return ESP_FAIL;
    }
    ESP_LOGI(ODRIVE_LOGGER_TAG,
             "ODrive discovered: Node ID: %d, Serial Number: "
             "%02x:%02x:%02x:%02x:%02x:%02x",
             context->node_id,
             address_response.serial_number[0],
             address_response.serial_number[1],
             address_response.serial_number[2],
             address_response.serial_number[3],
             address_response.serial_number[4],
             address_response.serial_number[5]);
    return ESP_OK;
}

esp_err_t odrive_get_axis_state(odrive_context_t *context, AxisState *axis_state) {
    if (context == NULL || axis_state == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context or axis_state pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (context->heartbeat_queue == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Heartbeat queue is NULL for Node ID %d", context->node_id);
        return ESP_FAIL;
    }

    odrive_stamped_payload_t stamped_payload;
    if (xQueuePeek(context->heartbeat_queue, &stamped_payload, pdMS_TO_TICKS(ODRIVE_HEARTBEAT_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to peek ODrive heartbeat queue for Node ID %d", context->node_id);
        return ESP_FAIL;
    }

    *axis_state = stamped_payload.payload.heartbeat.axis_state;
    return ESP_OK;
}

esp_err_t odrive_check_state(odrive_context_t *context, AxisState expected_axis_state) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    AxisState current_axis_state;
    if (odrive_get_axis_state(context, &current_axis_state) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to get ODrive axis state for Node ID %d", context->node_id);
        return ESP_FAIL;
    }
    if (current_axis_state != expected_axis_state) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t odrive_wait_state(odrive_context_t *context, AxisState target_axis_state, TickType_t timeout) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (context->heartbeat_queue == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Heartbeat queue is NULL for Node ID %d", context->node_id);
        return ESP_FAIL;
    }

    odrive_stamped_payload_t stamped_payload;
    if (xQueuePeek(context->heartbeat_queue, &stamped_payload, pdMS_TO_TICKS(ODRIVE_HEARTBEAT_TIMEOUT_MS)) == pdTRUE) {
        if (stamped_payload.payload.heartbeat.axis_state == target_axis_state) {
            ESP_LOGD(ODRIVE_LOGGER_TAG, "ODrive Node ID %d already in target state %d", context->node_id, target_axis_state);
            return ESP_OK;
        }
    }

    // Wait for the target state
    TickType_t start_time = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_time < timeout) {
        if (xQueueReceive(context->heartbeat_queue, &stamped_payload, timeout) == pdTRUE) {
            if (stamped_payload.payload.heartbeat.axis_state == target_axis_state) {
                ESP_LOGD(ODRIVE_LOGGER_TAG, "ODrive Node ID %d reached target state %d", context->node_id, target_axis_state);
                return ESP_OK;
            }
        }
    }
    ESP_LOGE(ODRIVE_LOGGER_TAG, "Timeout waiting for ODrive Node ID %d to reach state %d", context->node_id, target_axis_state);
    return ESP_ERR_TIMEOUT;
}

esp_err_t odrive_set_control_mode(odrive_context_t *context, ControlMode control_mode, InputMode input_mode) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    odrive_set_control_mode_t set_control_mode = {
        .control_mode = control_mode,
        .input_mode = input_mode,
    };
    uint32_t msg_id = odrive_can_pack_id(context->node_id, CMD_SET_CONTROL_MODE);

    ESP_LOGD(ODRIVE_LOGGER_TAG, "Sending SetControlMode (ID: 0x%03lX) for Node ID %d: Control=%d, Input=%d", msg_id, context->node_id, control_mode, input_mode);

    if (can_send_data(msg_id, &set_control_mode, sizeof(set_control_mode), pdMS_TO_TICKS(ODRIVE_CAN_MAX_TRANSMIT_QUEUE_TIMEOUT_MS)) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to send SetControlMode command for Node ID %d", context->node_id);
        return ESP_FAIL;
    }

    // Update the context with the new control mode and input mode
    context->control_mode = control_mode;
    context->input_mode = input_mode;
    ESP_LOGD(ODRIVE_LOGGER_TAG, "SetControlMode command sent successfully for Node ID %d (now in mode %d/%d)", context->node_id, control_mode, input_mode);
    return ESP_OK;
}

esp_err_t odrive_set_axis_state(odrive_context_t *context, AxisState axis_state, bool confirm) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    odrive_set_axis_state_t set_axis_state = {
        .axis_requested_state = axis_state,
    };
    uint32_t msg_id = odrive_can_pack_id(context->node_id, CMD_SET_AXIS_STATE);

    ESP_LOGI(ODRIVE_LOGGER_TAG, "Sending SetAxisState command (ID: 0x%03lX) for Node ID %d: State=%d", msg_id, context->node_id, axis_state);

    if (can_send_data(msg_id, &set_axis_state, sizeof(set_axis_state), pdMS_TO_TICKS(ODRIVE_CAN_MAX_TRANSMIT_QUEUE_TIMEOUT_MS)) != ESP_OK) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to send SetAxisState command for Node ID %d", context->node_id);
        return ESP_FAIL;
    }
    ESP_LOGD(ODRIVE_LOGGER_TAG, "SetAxisState command sent successfully for Node ID %d", context->node_id);

    if (confirm) {
        if (odrive_wait_state(context, axis_state, pdMS_TO_TICKS(S_TO_MS(ODRIVE_MAX_AXIS_STATE_CHANGE_TIME_S))) != ESP_OK) {
            ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to confirm SetAxisState command for Node ID %d", context->node_id);
            return ESP_FAIL;
        }
        ESP_LOGD(ODRIVE_LOGGER_TAG, "SetAxisState command confirmed for Node ID %d", context->node_id);
    }

    return ESP_OK;
}

// TODO: Add function to do a full calibration sequence

// TODO: Interface should be cleaned up
//! This function consumes the queue item
esp_err_t odrive_get_encoder_estimates(odrive_context_t *context, odrive_encoder_estimates_t *estimates, wallclock_timestamp_t *timestamp, bool consume, TickType_t timeout) {
    if (context == NULL || estimates == NULL || timestamp == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (context->encoder_estimates_queue == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Encoder estimates queue is NULL for Node ID %d", context->node_id);
        return ESP_FAIL;
    }

    odrive_stamped_payload_t stamped_payload;
    if (consume) {
        if (xQueueReceive(context->encoder_estimates_queue, &stamped_payload, timeout) != pdTRUE) {
            ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to receive encoder estimates for Node ID %d", context->node_id);
            return ESP_FAIL;
        }
    } else {
        if (xQueuePeek(context->encoder_estimates_queue, &stamped_payload, timeout) != pdTRUE) {
            ESP_LOGE(ODRIVE_LOGGER_TAG, "Failed to peek encoder estimates for Node ID %d", context->node_id);
            return ESP_FAIL;
        }
    }

    // TODO: Memcopy here?
    memcpy(estimates, &stamped_payload.payload.encoder_estimates, sizeof(odrive_encoder_estimates_t));
    memcpy(timestamp, &stamped_payload.wall_time, sizeof(wallclock_timestamp_t));
    return ESP_OK;
}

// TODO: This should probably be moved?
static uint32_t odrive_get_setpoint_command(ControlMode control_mode) {
    switch (control_mode) {
    case CONTROL_MODE_VELOCITY_CONTROL:
        return CMD_SET_INPUT_VEL;
    case CONTROL_MODE_POSITION_CONTROL:
        return CMD_SET_INPUT_POS;
    case CONTROL_MODE_TORQUE_CONTROL:
        return CMD_SET_INPUT_TORQUE;
    default:
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Invalid control mode for setpoint command");
        return UINT32_MAX; // Invalid command
    }
}

static bool odrive_get_setpoint_payload(ControlMode control_mode, const setpoint_payload_t *sp, const void **out_payload, size_t *out_len) {
    switch (control_mode) {
    case CONTROL_MODE_VELOCITY_CONTROL:
        *out_payload = &sp->vel;
        *out_len = sizeof(sp->vel);
        return true;

    case CONTROL_MODE_POSITION_CONTROL:
        *out_payload = &sp->pos;
        *out_len = sizeof(sp->pos);
        return true;

    case CONTROL_MODE_TORQUE_CONTROL:
        *out_payload = &sp->torque;
        *out_len = sizeof(sp->torque);
        return true;

    default:
        return false;
    }
}

static esp_err_t odrive_send_setpoint(odrive_context_t *context, setpoint_payload_t *setpoint) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    const void *payload;
    size_t len; // Should always be 8 bytes...
    if (!odrive_get_setpoint_payload(context->control_mode, setpoint, &payload, &len)) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Bad control_mode %d", context->control_mode);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t msg_ig = odrive_can_pack_id(context->node_id, odrive_get_setpoint_command(context->control_mode));
    return can_send_data(msg_ig, payload, len, pdMS_TO_TICKS(ODRIVE_CAN_MAX_TRANSMIT_QUEUE_TIMEOUT_MS));
}

// TODO: Not sure how to best structure the interface for this
esp_err_t odrive_set_velocity(odrive_context_t *context, float velocity, float torque_feed_forward) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (context->control_mode != CONTROL_MODE_VELOCITY_CONTROL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context is not in velocity control mode");
        return ESP_ERR_INVALID_STATE;
    }
    setpoint_payload_t sp = {
        .vel.input_vel = velocity * context->direction,
        .vel.input_torque_feed_forward = torque_feed_forward * context->direction, //? Do we need direction here?
    };
    return odrive_send_setpoint(context, &sp);
}

esp_err_t odrive_set_position(odrive_context_t *context, float position, int16_t velocity_feed_forward, int16_t torque_feed_forward) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (context->control_mode != CONTROL_MODE_POSITION_CONTROL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context is not in position control mode");
        return ESP_ERR_INVALID_STATE;
    }
    setpoint_payload_t sp = {
        .pos.input_pos = position * context->direction,                      //? Do we need direction here?
        .pos.vel_feed_forward = velocity_feed_forward * context->direction,  //? Do we need direction here?
        .pos.torque_feed_forward = torque_feed_forward * context->direction, //? Do we need direction here?
    };
    return odrive_send_setpoint(context, &sp);
}

esp_err_t odrive_set_torque(odrive_context_t *context, float torque) {
    if (context == NULL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (context->control_mode != CONTROL_MODE_TORQUE_CONTROL) {
        ESP_LOGE(ODRIVE_LOGGER_TAG, "Context is not in torque control mode");
        return ESP_ERR_INVALID_STATE;
    }
    setpoint_payload_t sp = {
        .torque.input_torque = torque * context->direction, //? Do we need direction here?
    };
    return odrive_send_setpoint(context, &sp);
}
