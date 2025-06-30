#ifndef ODRIVE_TYPES_H
#define ODRIVE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "utils/timing_utils.h"
#include <inttypes.h>
#include <stdint.h>

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-version
typedef struct __attribute__((packed)) {
    uint8_t protocol_version; // Always reported as 2
    uint8_t hw_version_major;
    uint8_t hw_version_minor;
    uint8_t hw_version_variant;
    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    uint8_t fw_version_revision;
    uint8_t fw_version_unreleased;
} odrive_version_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#heartbeat
typedef struct __attribute__((packed)) {
    uint32_t axis_error;
    uint8_t axis_state;
    uint8_t procedure_result;
    uint8_t traj_done_flag;
} odrive_heartbeat_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#estop
// Estop does not have a payload

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-error
typedef struct __attribute__((packed)) {
    uint32_t active_errors;
    uint32_t disarm_reason;
} odrive_error_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#rxsdo
typedef struct __attribute__((packed)) {
    uint8_t opcode;       // 0: read, 1: write
    uint16_t endpoint_id; // Endpoint ID as found in the flat_endpoints.json
                          // (download for apropriate firmware version here:
                          // https://docs.odriverobotics.com/releases/firmware)
    uint8_t reserved;     //?
    uint32_t value;       // Datatype and length depend on the endpoint
} odrive_rx_sdo_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#txsdo
typedef struct __attribute__((packed)) {
    uint8_t reserved0;    //?
    uint16_t endpoint_id; // Endpoint ID as found in the flat_endpoints.json
                          // (download for apropriate firmware version here:
                          // https://docs.odriverobotics.com/releases/firmware)
    uint8_t reserved1;    //?
    uint32_t value;       // Datatype and length depend on the endpoint
} odrive_tx_sdo_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#address
typedef struct __attribute__((packed)) {
    uint8_t node_id;
    uint8_t serial_number[6]; // 6 bytes; ODrive -> Host: Serial number of the sending
                              // ODrive; Host -> ODrive: The serial number that shall
                              // be associated with the specified Node_ID
} odrive_address_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-axis-state
typedef struct __attribute__((packed)) {
    uint32_t axis_requested_state;
} odrive_set_axis_state_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-encoder-estimates
typedef struct __attribute__((packed)) {
    float pos_estimate; // in revolutions; <axis>.pos_vel_mapper.pos_rel or
                        // <axis>.pos_vel_mapper.pos_abs, depending on
                        // ODrive.Controller.Config.absolute_setpoints
    float vel_estimate; // in revolutions per second;
} odrive_encoder_estimates_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-controller-mode
typedef struct __attribute__((packed)) {
    uint32_t control_mode; // See
                           // https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
    uint32_t input_mode;   // See
                           // https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
} odrive_set_control_mode_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-input-pos
typedef struct __attribute__((packed)) {
    float input_pos;             // in revolutions
    int16_t vel_feed_forward;    // in 0.001 rev/s (scale configurable)
    int16_t torque_feed_forward; // in 0.001 Nm
} odrive_set_input_pos_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-input-vel
typedef struct __attribute__((packed)) {
    float input_vel;                 // in rev/s
    float input_torque_feed_forward; // in Nm
} odrive_set_input_vel_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-input-torque
typedef struct __attribute__((packed)) {
    float input_torque; // in Nm
} odrive_set_input_torque_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-limits
typedef struct __attribute__((packed)) {
    float velocity_limit; // in rev/s
    float current_limit;  // in A
} odrive_set_limits_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-traj-vel-limit
typedef struct __attribute__((packed)) {
    float traj_vel_limit; // in rev/s
} odrive_set_traj_vel_limit_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-traj-accel-limit
typedef struct __attribute__((packed)) {
    float traj_accel_limit; // in rev/s^2
    float traj_decel_limit; // in rev/s^2
} odrive_set_traj_accel_limit_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-traj-inertia
typedef struct __attribute__((packed)) {
    float traj_inertia; // in Nm/(rev/s^2)
} odrive_set_traj_inertia_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-iq
typedef struct __attribute__((packed)) {
    float iq_setpoint; // in A
    float iq_measured; // in A
} odrive_iq_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-temperature
typedef struct __attribute__((packed)) {
    float fet_temperture;    // in °C
    float motor_temperature; // in °C
} odrive_temperature_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#reboot
typedef struct __attribute__((packed)) {
    uint8_t action; // 0: Reboot, 1: Save config, 2: Erase config, 3: Enter DFU mode
} odrive_reboot_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-bus-voltage-current
typedef struct __attribute__((packed)) {
    float bus_voltage; // in V
    float bus_current; // in A
} odrive_bus_voltage_current_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#clear-errors
typedef struct __attribute__((packed)) {
    uint8_t identify; //?
} odrive_clear_errors_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-absolute-position
typedef struct __attribute__((packed)) {
    float position; // in revolutions
} odrive_set_absolute_position_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-pos-gain
typedef struct __attribute__((packed)) {
    float pos_gain; // in (revs/s) / rev
} odrive_set_pos_gain_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#set-vel-gains
typedef struct __attribute__((packed)) {
    float vel_gain;            // in Nm / (rev/s)
    float vel_integrator_gain; // in Nm / rev
} odrive_set_vel_gains_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-torques
typedef struct __attribute__((packed)) {
    float torque_target;   // in Nm
    float torque_estimate; // in Nm
} odrive_torques_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#get-powers
typedef struct __attribute__((packed)) {
    float electrical_power; // in W
    float mechanical_power; // in W
} odrive_powers_t;

// https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#enter-dfu-mode
// Enter DFU mode does not have a payload

// union to hold all possible payloads
typedef union {
    odrive_version_t version;
    odrive_heartbeat_t heartbeat;
    odrive_error_t error;
    odrive_rx_sdo_t rxsdo;
    odrive_tx_sdo_t txsdo;
    odrive_address_t address;
    odrive_set_axis_state_t set_axis_state;
    odrive_encoder_estimates_t encoder_estimates;
    odrive_set_control_mode_t set_control_mode;
    odrive_set_input_pos_t set_input_pos;
    odrive_set_input_vel_t set_input_vel;
    odrive_set_input_torque_t set_input_torque;
    odrive_set_limits_t set_limits;
    odrive_set_traj_vel_limit_t set_traj_vel_limit;
    odrive_set_traj_accel_limit_t set_traj_accel_limit;
    odrive_set_traj_inertia_t set_traj_inertia;
    odrive_iq_t iq;
    odrive_temperature_t temperature;
    odrive_reboot_t reboot;
    odrive_bus_voltage_current_t bus_voltage_current;
    odrive_clear_errors_t clear_errors;
    odrive_set_absolute_position_t set_absolute_position;
    odrive_set_pos_gain_t set_pos_gain;
    odrive_set_vel_gains_t set_vel_gains;
    odrive_torques_t torques;
    odrive_powers_t powers;
} odrive_payload_t;

typedef struct {
    odrive_payload_t payload;
    wallclock_timestamp_t wall_time;      // Synchronized with the micro-ROS agent -> dont use for timeouts, as it might jump
    monotonic_timestamp_t monotonic_time; // Monotonic time since boot -> use for timeouts
} odrive_stamped_payload_t;

typedef union {
    odrive_set_input_vel_t vel;
    odrive_set_input_pos_t pos;
    odrive_set_input_torque_t torque;
} setpoint_payload_t;

#ifdef __cplusplus
}
#endif

#endif // ODRIVE_TYPES_H