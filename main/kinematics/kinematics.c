#include "kinematics.h"
#include "mros/mros_param.h"
#include "utils/math_utils.h"
#include "utils/timing_utils.h"

#include <math.h>
#include <string.h>

static robot_parameters_t local_params_mm = {0};
static float last_steer = 0.0f;

esp_err_t forward_kinematics(
    float in_delta_pos_l, float in_delta_pos_r, float in_vel_l, float in_vel_r, float *out_pos_x, float *out_pos_y, float *out_qx, float *out_qy, float *out_qz, float *out_qw, float *out_vel_x, float *out_omega_z) {
    if (!out_pos_x || !out_pos_y || !out_qx || !out_qy || !out_qz || !out_qw || !out_vel_x || !out_omega_z) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1) Pull the latest parameters:
    if (robot_parameters_get(&local_params_mm) != ESP_OK) {
        // No parameters yet
        return ESP_ERR_INVALID_STATE;
    }

    // 2) Convert all relevant dims to meters:
    float wheel_radius_m = MM_TO_M(local_params_mm.wheel_radius);
    float track_width_m = MM_TO_M(local_params_mm.track_width); // rear‐axle b
    float wheel_base_m = MM_TO_M(local_params_mm.wheel_base);   // L

    // 3) Compute linear rear‐wheel displacements:
    //    delta_s_l = in_delta_pos_l * r
    //    delta_s_r = in_delta_pos_r * r
    float delta_pos_l_m = in_delta_pos_l * wheel_radius_m;
    float delta_pos_r_m = in_delta_pos_r * wheel_radius_m;
    // Midpoint rear travel:
    float delta_s = 0.5f * (delta_pos_l_m + delta_pos_r_m);

    // 4) Compute delta_θ from front‐steer (bicycle‐model):
    //    delta_θ = delta_s * (tan d) / L
    float delta_theta;
    float tan_delta = tanf(last_steer);
    if (fabsf(tan_delta) < 1e-6f) {
        // Treat as straight → no yaw change
        delta_theta = 0.0f;
    } else {
        delta_theta = delta_s * (tan_delta / wheel_base_m);
    }

    // 5) Unpack previous yaw from the quaternion:
    float roll, pitch, yaw;
    math_quaternion_to_euler(*out_qx, *out_qy, *out_qz, *out_qw, &roll, &pitch, &yaw);

    // 6) Midpoint heading for XY integration:
    float mid_yaw = yaw + 0.5f * delta_theta;
    *out_pos_x += delta_s * cosf(mid_yaw);
    *out_pos_y += delta_s * sinf(mid_yaw);

    // 7) Compute new yaw and pack back into quaternion:
    float new_yaw = math_normalize_angle(yaw + delta_theta);
    math_euler_to_quaternion(0.0f, 0.0f, new_yaw, out_qx, out_qy, out_qz, out_qw);

    // 8) Body‐frame velocities:
    //    v_l = in_vel_l * r, v_r = in_vel_r * r
    float v_l = in_vel_l * wheel_radius_m;
    float v_r = in_vel_r * wheel_radius_m;
    //    v_CB = (v_l + v_r)/2:
    float v_CB = 0.5f * (v_l + v_r);
    //    omega_chassis = v_CB * tan(d) / L
    float omega_chassis;
    if (fabsf(tan_delta) < 1e-6f) {
        omega_chassis = 0.0f;
    } else {
        omega_chassis = v_CB * (tan_delta / wheel_base_m);
    }

    *out_vel_x = v_CB;
    *out_omega_z = omega_chassis;

    return ESP_OK;
}

esp_err_t inverse_kinematics(float in_vel_x, float in_omega_z, float *out_vel_r, float *out_vel_l, float *out_steer) {
    if (!out_vel_r || !out_vel_l || !out_steer) {
        return ESP_ERR_INVALID_ARG;
    }

    // 1) Pull parameters:
    if (robot_parameters_get(&local_params_mm) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    float wheel_radius_m = MM_TO_M(local_params_mm.wheel_radius);
    float track_width_m = MM_TO_M(local_params_mm.track_width); // b
    float wheel_base_m = MM_TO_M(local_params_mm.wheel_base);   // L

    // 2) Compute d = atan(L * omega / v):
    float d;
    if (fabsf(in_vel_x) < 1e-6f) {
        // Allow steering even when the vehicle is stationary by mapping the
        // desired yaw rate directly to a steering angle. For small yaw rates
        // this approximates d ~ L * omega.
        d = atanf(wheel_base_m * in_omega_z);
        *out_vel_l = 0.0f;
        *out_vel_r = 0.0f;
    } else {
        d = atanf((wheel_base_m * in_omega_z) / in_vel_x);
    }
    *out_steer = d;
    last_steer = d;

    // 3) Compute turning radius R = L / tan(d):
    float tan_delta = tanf(d);
    float R_CB;
    if (fabsf(tan_delta) < 1e-6f) {
        // Near straight:
        R_CB = 1e9f; // effectively infinite
    } else {
        R_CB = wheel_base_m / tan_delta;
    }

    // 4) Compute rear‐wheel linear speeds:
    //    v_l = in_vel_x * (1 − b/(2 R_CB))
    //    v_r = in_vel_x * (1 + b/(2 R_CB))
    float v_l_lin, v_r_lin;
    if (fabsf(in_omega_z) < 1e-6f || fabsf(tan_delta) < 1e-6f) {
        // Straight line:
        v_l_lin = in_vel_x;
        v_r_lin = in_vel_x;
    } else {
        float ratio = (track_width_m / (2.0f * R_CB));
        v_l_lin = in_vel_x * (1.0f - ratio);
        v_r_lin = in_vel_x * (1.0f + ratio);
    }

    // 5) Convert to wheel angular speeds [rad/s]:
    *out_vel_l = v_l_lin / wheel_radius_m;
    *out_vel_r = v_r_lin / wheel_radius_m;

    return ESP_OK;
}
