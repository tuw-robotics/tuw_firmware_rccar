#ifndef BNO_WRAPPER_H
#define BNO_WRAPPER_H

#include <esp_err.h>
#include <stdint.h>

typedef enum {
    BNO055_OPERATION_MODE_CONFIG = 0x00,
    BNO055_OPERATION_MODE_ACCONLY = 0x01,
    BNO055_OPERATION_MODE_MAGONLY = 0x02,
    BNO055_OPERATION_MODE_GYRONLY = 0x03,
    BNO055_OPERATION_MODE_ACCMAG = 0x04,
    BNO055_OPERATION_MODE_ACCGYRO = 0x05,
    BNO055_OPERATION_MODE_MAGGYRO = 0x06,
    BNO055_OPERATION_MODE_AMG = 0x07,
    BNO055_OPERATION_MODE_IMU = 0x08,
    BNO055_OPERATION_MODE_COMPASS = 0x09,
    BNO055_OPERATION_MODE_M4G = 0x0A,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPERATION_MODE_NDOF = 0x0C
} bno_opmode_t;

typedef struct {
    double x;
    double y;
    double z;
} bno_vector_t;

typedef struct {
    double w;
    double x;
    double y;
    double z;
} bno_quaternion_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the BNO055 sensor on the specified I2C port and address.
 *
 * @param i2c_port I2C port number.
 * @param address I2C address of the BNO055 sensor.
 * @return ESP_OK on success, or an error code from esp_err_t.
 */
esp_err_t bno_begin_i2c(int i2c_port, uint8_t address);

/**
 * @brief Set the operation mode of the BNO055 sensor.
 *
 * @param mode Operation mode (see bno_opmode_t).
 * @return ESP_OK on success, or an error code from esp_err_t.
 */
esp_err_t bno_set_operation_mode(bno_opmode_t mode);

/**
 * @brief Get the acceleration vector from the sensor.
 *
 * @return Acceleration as a bno_vector_t.
 */
bno_vector_t bno_get_acceleration();

/**
 * @brief Get the magnetic field vector from the sensor.
 *
 * @return Magnetic field as a bno_vector_t.
 */
bno_vector_t bno_get_magnetic_field();

/**
 * @brief Get the angular velocity vector from the sensor.
 *
 * @return Angular velocity as a bno_vector_t.
 */
bno_vector_t bno_get_angular_velocity();

/**
 * @brief Get the Euler angles (heading, roll, pitch in degrees).
 *
 * @return Euler angles as a bno_vector_t.
 */
bno_vector_t bno_get_euler_angles();

/**
 * @brief Get the orientation as a quaternion.
 *
 * @return Orientation as a bno_quaternion_t.
 */
bno_quaternion_t bno_get_quaternion();

/**
 * @brief Get the linear acceleration vector.
 *
 * @return Linear acceleration as a bno_vector_t.
 */
bno_vector_t bno_get_linear_acceleration();

/**
 * @brief Get the gravity vector.
 *
 * @return Gravity vector as a bno_vector_t.
 */
bno_vector_t bno_get_gravity_vector();

/**
 * @brief Get the temperature from the sensor.
 *
 * @return Temperature as int8_t.
 */
int8_t bno_get_temperature();

/**
 * @brief Stop communication with the sensor and release resources.
 *
 * @return ESP_OK on success, or an error code from esp_err_t.
 */
esp_err_t bno_stop();

#ifdef __cplusplus
}
#endif

#endif // BNO_WRAPPER_H