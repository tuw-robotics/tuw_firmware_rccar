#ifndef MROS_PARAMETERS_H
#define MROS_PARAMETERS_H

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <rclc_parameter/rclc_parameter.h>
#include <sdkconfig.h>

#ifdef CONFIG_ROBOT_WHEEL_RADIUS_MM
#define ROBOT_WHEEL_RADIUS_MM CONFIG_ROBOT_WHEEL_RADIUS_MM
#else
#define ROBOT_WHEEL_RADIUS_MM 32
#endif

#ifdef CONFIG_ROBOT_WHEEL_RADIUS_PARAM_NAME
#define ROBOT_WHEEL_RADIUS_PARAM_NAME CONFIG_ROBOT_WHEEL_RADIUS_PARAM_NAME
#else
//! NVS keys may not exceed 15 characters!
#define ROBOT_WHEEL_RADIUS_PARAM_NAME "whl_radius_mm"
#endif

#ifdef CONFIG_ROBOT_TRACK_WIDTH_MM
#define ROBOT_TRACK_WIDTH_MM CONFIG_ROBOT_TRACK_WIDTH_MM
#else
#define ROBOT_TRACK_WIDTH_MM 185
#endif

#ifdef CONFIG_ROBOT_TRACK_WIDTH_PARAM_NAME
#define ROBOT_TRACK_WIDTH_PARAM_NAME CONFIG_ROBOT_TRACK_WIDTH_PARAM_NAME
#else
#define ROBOT_TRACK_WIDTH_PARAM_NAME "trk_width_mm"
#endif

#ifdef CONFIG_ROBOT_WHEEL_BASE_MM
#define ROBOT_WHEEL_BASE_MM CONFIG_ROBOT_WHEEL_BASE_MM
#else
#define ROBOT_WHEEL_BASE_MM 255
#endif

#ifdef CONFIG_ROBOT_WHEEL_BASE_PARAM_NAME
#define ROBOT_WHEEL_BASE_PARAM_NAME CONFIG_ROBOT_WHEEL_BASE_PARAM_NAME
#else
#define ROBOT_WHEEL_BASE_PARAM_NAME "whl_base_mm"
#endif

#ifdef CONFIG_MAX_ANG_VEL
#define MAX_ANG_VEL CONFIG_MAX_ANG_VEL
#else
#define MAX_ANG_VEL 2000
#endif
#define MAX_ANG_VEL_PARAM_NAME "max_ang_vel"

#ifdef CONFIG_CORR_WEIGHT
#define CORR_WEIGHT CONFIG_CORR_WEIGHT
#else
#define CORR_WEIGHT 0
#endif
#define CORR_WEIGHT_PARAM_NAME "corr_weight"

#ifdef CONFIG_DEADBEAT_START
#define DEADBEAT_START CONFIG_DEADBEAT_START
#else
#define DEADBEAT_START 35
#endif
#define DEADBEAT_START_PARAM_NAME "deadbeat_start"

#ifdef CONFIG_DEADBEAT_END
#define DEADBEAT_END CONFIG_DEADBEAT_END
#else
#define DEADBEAT_END 26
#endif
#define DEADBEAT_END_PARAM_NAME "deadbeat_end"

// Check if names are too long
_Static_assert(sizeof(ROBOT_WHEEL_RADIUS_PARAM_NAME) <= 15, "ROBOT_WHEEL_RADIUS_PARAM_NAME must not exceed 15 characters");
_Static_assert(sizeof(ROBOT_TRACK_WIDTH_PARAM_NAME) <= 15, "ROBOT_TRACK_WIDTH_PARAM_NAME must not exceed 15 characters");
_Static_assert(sizeof(ROBOT_WHEEL_BASE_PARAM_NAME) <= 15, "ROBOT_WHEEL_BASE_PARAM_NAME must not exceed 15 characters");
_Static_assert(sizeof(MAX_ANG_VEL_PARAM_NAME) <= 15, "MAX_ANG_VEL_PARAM_NAME must not exceed 15 characters");
_Static_assert(sizeof(CORR_WEIGHT_PARAM_NAME) <= 15, "CORR_WEIGHT_PARAM_NAME must not exceed 15 characters");
_Static_assert(sizeof(DEADBEAT_START_PARAM_NAME) <= 15, "DEADBEAT_START_PARAM_NAME must not exceed 15 characters");
_Static_assert(sizeof(DEADBEAT_END_PARAM_NAME) <= 15, "DEADBEAT_END_PARAM_NAME must not exceed 15 characters");

typedef struct {
    int32_t track_width;
    int32_t wheel_radius;
    int32_t wheel_base;
    float max_angular_velocity;
    float correction_weight;
    float deadbeat_start;
    float deadbeat_end;
} robot_parameters_t;

/**
 * @brief Initializes the robot parameters module
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t robot_parameters_init(void);

/**
 * @brief Registers all robot parameters with the parameter server
 *
 * @param server Pointer to the parameter server to register the parameters with
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t robot_parameters_register_all(rclc_parameter_server_t *server);

/**
 * @brief Handles a change in robot parameters from ROS
 *
 * @param new_param Pointer to the new parameter that has changed
 * @return true if the parameter was successfully handled and updated,
 * @return false if the parameter was not recognized or could not be handled
 */
bool robot_parameters_handle_ros_change(const rcl_interfaces__msg__Parameter *new_param);

/**
 * @brief Gets the current robot parameters
 *
 * @param params Pointer to a robot_parameters_t structure to store the parameters
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params is NULL, ESP_ERR_INVALID_STATE if parameters are not available
 */
esp_err_t robot_parameters_get(robot_parameters_t *params);

/**
 * @brief Gets the preconfigured robot parameters
 *
 * @param params Pointer to a robot_parameters_t structure to store the parameters
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if params is NULL
 */
esp_err_t robot_parameters_get_preconfigured(robot_parameters_t *params);

#endif // MROS_PARAMETERS_H
