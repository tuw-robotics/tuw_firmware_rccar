#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>

#define MIN_3(a, b, c) (MIN(a, b) < (c) ? MIN(a, b) : (c))
#define MAX_3(a, b, c) (MAX(a, b) > (c) ? MAX(a, b) : (c))

#define RAD_TO_REV(a) ((a) / (2.0f * M_PI))
#define REV_TO_RAD(a) ((a) * (2.0f * M_PI))
#define RAD_TO_DEG(a) ((a) * (180.0f / M_PI))
#define DEG_TO_RAD(a) ((a) * (M_PI / 180.0f))

#define MM_TO_M(a) ((a) / 1000.0f)
#define M_TO_MM(a) ((a) * 1000.0f)

/**
 * @brief Convert Euler angles (roll, pitch, yaw) to a quaternion representation
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @param qx
 * @param qy
 * @param qz
 * @param qw
 */
void math_euler_to_quaternion(float roll, float pitch, float yaw, float *qx, float *qy, float *qz, float *qw);

/**
 * @brief Convert a quaternion representation to Euler angles (roll, pitch, yaw)
 *
 * @param qx
 * @param qy
 * @param qz
 * @param qw
 * @param roll
 * @param pitch
 * @param yaw
 */
void math_quaternion_to_euler(float qx, float qy, float qz, float qw, float *roll, float *pitch, float *yaw);

/**
 * @brief Normalize an angle to the range [-pi, pi]
 *
 * @param angle Angle in radians
 * @return float Normalized angle in radians
 */
float math_normalize_angle(float angle);

#endif // MATH_UTILS_H