#include "bno_wrapper.h"
#include "BNO055ESP32.h"
#include <esp_err.h>
#include <stdint.h>

static BNO055 *bno = nullptr;

extern "C" {

esp_err_t bno_begin_i2c(int i2c_port, uint8_t address) {
    if (bno) {
        try {
            bno->stop();
        } catch (...) {
        }
        delete bno;
        bno = nullptr;
    }

    try {
        bno = new BNO055((i2c_port_t)i2c_port, address);
        bno->begin();
        return ESP_OK;
    } catch (...) {
        if (bno) {
            delete bno;
            bno = nullptr;
        }
        return ESP_FAIL;
    }
}

esp_err_t bno_set_operation_mode(bno_opmode_t mode) {
    if (!bno) {
        return ESP_FAIL;
    }
    try {
        switch (mode) {
        case BNO055_OPERATION_MODE_CONFIG:
            bno->setOprModeConfig();
            break;
        case BNO055_OPERATION_MODE_ACCONLY:
            bno->setOprModeAccOnly();
            break;
        case BNO055_OPERATION_MODE_MAGONLY:
            bno->setOprModeMagOnly();
            break;
        case BNO055_OPERATION_MODE_GYRONLY:
            bno->setOprModeGyroOnly();
            break;
        case BNO055_OPERATION_MODE_ACCMAG:
            bno->setOprModeAccMag();
            break;
        case BNO055_OPERATION_MODE_ACCGYRO:
            bno->setOprModeAccGyro();
            break;
        case BNO055_OPERATION_MODE_MAGGYRO:
            bno->setOprModeMagGyro();
            break;
        case BNO055_OPERATION_MODE_AMG:
            bno->setOprModeAMG();
            break;
        case BNO055_OPERATION_MODE_IMU:
            bno->setOprModeIMU();
            break;
        case BNO055_OPERATION_MODE_COMPASS:
            bno->setOprModeCompass();
            break;
        case BNO055_OPERATION_MODE_M4G:
            bno->setOprModeM4G();
            break;
        case BNO055_OPERATION_MODE_NDOF_FMC_OFF:
            bno->setOprModeNdofFmcOff();
            break;
        case BNO055_OPERATION_MODE_NDOF:
            bno->setOprModeNdof();
            break;
        default:
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    } catch (...) {
        return ESP_FAIL;
    }
}

bno_vector_t bno_get_acceleration() {
    if (!bno) {
        return bno_vector_t({0, 0, 0});
    }
    try {
        bno055_vector_t vec = bno->getVectorAccelerometer();
        bno_vector_t result;
        result.x = vec.x;
        result.y = vec.y;
        result.z = vec.z;
        return result;
    } catch (...) {
        return bno_vector_t({0, 0, 0});
    }
}

bno_vector_t bno_get_magnetic_field() {
    if (!bno) {
        return bno_vector_t({0, 0, 0});
    }
    try {
        bno055_vector_t vec = bno->getVectorMagnetometer();
        bno_vector_t result;
        result.x = vec.x;
        result.y = vec.y;
        result.z = vec.z;
        return result;
    } catch (...) {
        return bno_vector_t({0, 0, 0});
    }
}

bno_vector_t bno_get_angular_velocity() {
    if (!bno) {
        return bno_vector_t({0, 0, 0});
    }
    try {
        bno055_vector_t vec = bno->getVectorGyroscope();
        bno_vector_t result;
        result.x = vec.x;
        result.y = vec.y;
        result.z = vec.z;
        return result;
    } catch (...) {
        return bno_vector_t({0, 0, 0});
    }
}

bno_vector_t bno_get_euler_angles() {
    if (!bno) {
        return bno_vector_t({0, 0, 0});
    }
    try {
        bno055_vector_t vec = bno->getVectorEuler();
        bno_vector_t result;
        result.x = vec.x;
        result.y = vec.y;
        result.z = vec.z;
        return result;
    } catch (...) {
        return bno_vector_t({0, 0, 0});
    }
}

bno_quaternion_t bno_get_quaternion() {
    if (!bno) {
        return bno_quaternion_t({0, 0, 0, 0});
    }
    try {
        bno055_quaternion_t quat = bno->getQuaternion();
        bno_quaternion_t result;
        result.w = quat.w;
        result.x = quat.x;
        result.y = quat.y;
        result.z = quat.z;
        return result;
    } catch (...) {
        return bno_quaternion_t({0, 0, 0, 0});
    }
}

bno_vector_t bno_get_linear_acceleration() {
    if (!bno) {
        return bno_vector_t({0, 0, 0});
    }
    try {
        bno055_vector_t vec = bno->getVectorLinearAccel();
        bno_vector_t result;
        result.x = vec.x;
        result.y = vec.y;
        result.z = vec.z;
        return result;
    } catch (...) {
        return bno_vector_t({0, 0, 0});
    }
}

bno_vector_t bno_get_gravity_vector() {
    if (!bno) {
        return bno_vector_t({0, 0, 0});
    }
    try {
        bno055_vector_t vec = bno->getVectorGravity();
        bno_vector_t result;
        result.x = vec.x;
        result.y = vec.y;
        result.z = vec.z;
        return result;
    } catch (...) {
        return bno_vector_t({0, 0, 0});
    }
}

int8_t bno_get_temperature() {
    if (!bno) {
        return 0;
    }
    try {
        return bno->getTemp();
    } catch (...) {
        return 0;
    }
}

esp_err_t bno_stop() {
    if (!bno) {
        return ESP_FAIL;
    }
    try {
        bno->stop();
        delete bno;
        bno = nullptr;
        return ESP_OK;
    } catch (...) {
        return ESP_FAIL;
    }
}

} // extern "C"