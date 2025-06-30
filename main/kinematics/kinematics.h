#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <esp_err.h>

// TODO: These method signatures should be improved!
/**
 * @brief Forward kinematics calculation for a ackermann steering vehicle
 *
 * @param in_pos_l delta position of the left rear wheel [rad]
 * @param in_pos_r delta position of the right rear wheel [rad]
 * @param in_vel_l current velocity of the left rear wheel [rad/s]
 * @param in_vel_r current velocity of the right rear wheel [rad/s]
 * @param out_pos_x chassis X position [m], updated in place
 * @param out_pos_y chassis Y position [m], updated in place
 * @param out_qx rotation quaternion x, updated in place
 * @param out_qy rotation quaternion y
 * @param out_qz rotation quaternion z
 * @param out_qw rotation quaternion w
 * @param out_vel_x body‐frame forward speed [m/s]
 * @param out_omega_z body‐frame yaw rate [rad/s]
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if any output pointer is NULL, ESP_ERR_INVALID_STATE if parameters are not available
 */
esp_err_t
forward_kinematics(float in_pos_l, float in_pos_r, float in_vel_l, float in_vel_r, float *out_pos_x, float *out_pos_y, float *out_qx, float *out_qy, float *out_qz, float *out_qw, float *out_vel_x, float *out_omega_z);

/**
 * @brief Inverse kinematics calculation for a ackermann steering vehicle
 *
 * The calculations for steering angle are based on differential drive kinematics of the back wheels -> steering angle follows the back wheels
 *
 * @param in_vel_x desired chassis forward speed [m/s]
 * @param in_omega_z desired chassis yaw rate [rad/s]
 * @param out_vel_r right rear wheel omega [rad/s]
 * @param out_vel_l left rear wheel omega [rad/s]
 * @param out_steer front steer d [rad]
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if any output pointer is NULL, ESP_ERR_INVALID_STATE if parameters are not available
 */
esp_err_t inverse_kinematics(float in_vel_x, float in_omega_z, float *out_vel_r, float *out_vel_l, float *out_steer);

#endif // KINEMATICS_H