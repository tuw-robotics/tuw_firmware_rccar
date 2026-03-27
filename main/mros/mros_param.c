#include "mros_param.h"
#include "mros_conf.h"
#include "utils/timing_utils.h"
#include <esp_log.h>
#include <inttypes.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <sdkconfig.h>
#include <string.h>

#define PARAM_NAMESPACE "robot_params"

static robot_parameters_t robot_parameters = {0};
QueueHandle_t robot_params_queue = NULL;

//! Double parameters cannot be handled nicely within RISC-V, so we need to stick to int32_t for now

static esp_err_t load_int_from_nvs(int32_t *out_value, const char *key) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_get_i32(nvs_handle, key, out_value);
    nvs_close(nvs_handle);
    return err;
}

static esp_err_t save_int_to_nvs(int32_t value, const char *key) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_i32(nvs_handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return err;
}

esp_err_t robot_parameters_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to init NVS");
        return err;
    }

    robot_params_queue = xQueueCreate(1, sizeof(robot_parameters_t));
    if (robot_params_queue == NULL) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to create params queue");
        return ESP_ERR_NO_MEM;
    }

    // Wheel radius
    int32_t wheel_radius_mm;
    if (load_int_from_nvs(&wheel_radius_mm, ROBOT_WHEEL_RADIUS_PARAM_NAME) == ESP_OK) {
        robot_parameters.wheel_radius = wheel_radius_mm;
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded wheel_radius = %ld mm from NVS", robot_parameters.wheel_radius);
    } else {
        robot_parameters.wheel_radius = ROBOT_WHEEL_RADIUS_MM;
        ESP_LOGW(MROS_LOGGER_TAG, "Using default wheel_radius = %ld mm", robot_parameters.wheel_radius);
    }

    // Track width
    int32_t track_width_mm;
    if (load_int_from_nvs(&track_width_mm, ROBOT_TRACK_WIDTH_PARAM_NAME) == ESP_OK) {
        robot_parameters.track_width = track_width_mm;
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded track_width = %ld mm from NVS", robot_parameters.track_width);
    } else {
        robot_parameters.track_width = ROBOT_TRACK_WIDTH_MM;
        ESP_LOGW(MROS_LOGGER_TAG, "Using default track_width = %ld mm", robot_parameters.track_width);
    }

    // Wheel base
    int32_t wheel_base_mm;
    if (load_int_from_nvs(&wheel_base_mm, ROBOT_WHEEL_BASE_PARAM_NAME) == ESP_OK) {
        robot_parameters.wheel_base = wheel_base_mm;
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded wheel_base = %ld mm from NVS", robot_parameters.wheel_base);
    } else {
        robot_parameters.wheel_base = ROBOT_WHEEL_BASE_MM;
        ESP_LOGW(MROS_LOGGER_TAG, "Using default wheel_base = %ld mm", robot_parameters.wheel_base);
    }

    int32_t max_ang_vel;
    if (load_int_from_nvs(&max_ang_vel, MAX_ANG_VEL_PARAM_NAME) == ESP_OK) {
        robot_parameters.max_angular_velocity = (float)max_ang_vel / 1000.0f; // Convert back to float
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded max_angular_velocity = %.3f from NVS", robot_parameters.max_angular_velocity);
    } else {
        robot_parameters.max_angular_velocity = (float)MAX_ANG_VEL / 1000.0f; // Convert back to float
        ESP_LOGW(MROS_LOGGER_TAG, "Using default max_angular_velocity = %.3f", robot_parameters.max_angular_velocity);
    }

    int32_t corr_weight;
    if (load_int_from_nvs(&corr_weight, CORR_WEIGHT_PARAM_NAME) == ESP_OK) {
        robot_parameters.correction_weight = (float)corr_weight / 1000.0f; // Convert back to float
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded correction_weight = %.3f from NVS", robot_parameters.correction_weight);
    } else {
        robot_parameters.correction_weight = (float)CORR_WEIGHT / 1000.0f; // Convert back to float
        ESP_LOGW(MROS_LOGGER_TAG, "Using default correction_weight = %.3f", robot_parameters.correction_weight);
    }

    int32_t db_start;
    if (load_int_from_nvs(&db_start, DEADBEAT_START_PARAM_NAME) == ESP_OK) {
        robot_parameters.deadbeat_start = (float)db_start / 1000.0f; // Convert back to float
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded deadbeat_start = %.3f from NVS", robot_parameters.deadbeat_start);
    } else {
        robot_parameters.deadbeat_start = (float)DEADBEAT_START / 1000.0f; // Convert back to float
        ESP_LOGW(MROS_LOGGER_TAG, "Using default deadbeat_start = %.3f", robot_parameters.deadbeat_start);
    }

    int32_t db_end;
    if (load_int_from_nvs(&db_end, DEADBEAT_END_PARAM_NAME) == ESP_OK) {
        robot_parameters.deadbeat_end = (float)db_end / 1000.0f; // Convert back to float
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded deadbeat_end = %.3f from NVS", robot_parameters.deadbeat_end);
    } else {
        robot_parameters.deadbeat_end = (float)DEADBEAT_END / 1000.0f; // Convert back to float
        ESP_LOGW(MROS_LOGGER_TAG, "Using default deadbeat_end = %.3f", robot_parameters.deadbeat_end);
    }

    int32_t kp;
    if (load_int_from_nvs(&kp, KP_PARAM_NAME) == ESP_OK) {
        robot_parameters.kp = (float)kp / 1000.0f; // Convert back to float
        ESP_LOGI(MROS_LOGGER_TAG, "Loaded kp = %.3f from NVS", robot_parameters.kp);
    } else {
        robot_parameters.kp = (float)KP / 1000.0f; // Convert back to float
        ESP_LOGW(MROS_LOGGER_TAG, "Using default kp = %.3f", robot_parameters.kp);
    }

    xQueueOverwrite(robot_params_queue, &robot_parameters);

    return ESP_OK;
}

esp_err_t robot_parameters_register_all(rclc_parameter_server_t *server) {
    rcl_ret_t rc;

    rc = rclc_add_parameter(server, ROBOT_WHEEL_RADIUS_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add wheel radius parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, ROBOT_WHEEL_RADIUS_PARAM_NAME, robot_parameters.wheel_radius);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for wheel radius");
        return ESP_FAIL;
    }

    rc = rclc_add_parameter(server, ROBOT_TRACK_WIDTH_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add track width parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, ROBOT_TRACK_WIDTH_PARAM_NAME, robot_parameters.track_width);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for track width");
        return ESP_FAIL;
    }

    rc = rclc_add_parameter(server, ROBOT_WHEEL_BASE_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add wheel base parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, ROBOT_WHEEL_BASE_PARAM_NAME, robot_parameters.wheel_base);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for wheel base");
        return ESP_FAIL;
    }

    rc = rclc_add_parameter(server, MAX_ANG_VEL_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add max angular velocity parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, MAX_ANG_VEL_PARAM_NAME, (int32_t)(robot_parameters.max_angular_velocity * 1000)); // Convert to int
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for max angular velocity");
        return ESP_FAIL;
    }

    rc = rclc_add_parameter(server, CORR_WEIGHT_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add correction weight parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, CORR_WEIGHT_PARAM_NAME, (int32_t)(robot_parameters.correction_weight * 1000)); // Convert to int
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for correction weight");
        return ESP_FAIL;
    }

    rc = rclc_add_parameter(server, DEADBEAT_START_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add deadbeat start parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, DEADBEAT_START_PARAM_NAME, (int32_t)(robot_parameters.deadbeat_start * 1000)); // Convert to int
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for deadbeat start");
        return ESP_FAIL;
    }

    rc = rclc_add_parameter(server, DEADBEAT_END_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add deadbeat end parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, DEADBEAT_END_PARAM_NAME, (int32_t)(robot_parameters.deadbeat_end * 1000)); // Convert to int
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for deadbeat end");
        return ESP_FAIL;
    }

    rc = rclc_add_parameter(server, KP_PARAM_NAME, RCLC_PARAMETER_INT);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to add kp parameter");
        return ESP_FAIL;
    }
    rc = rclc_parameter_set_int(server, KP_PARAM_NAME, (int32_t)(robot_parameters.kp * 1000)); // Convert to int
    if (rc != RCL_RET_OK) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to set initial value for kp");
        return ESP_FAIL;
    }

    ESP_LOGI(MROS_LOGGER_TAG, "Added all parameters to parameter server");

    return ESP_OK;
}

bool robot_parameters_handle_ros_change(const rcl_interfaces__msg__Parameter *new_param) {
    if (!new_param) {
        ESP_LOGW(MROS_LOGGER_TAG, "NULL parameter change received");
        return false;
    }

    if (strcmp(new_param->name.data, ROBOT_WHEEL_RADIUS_PARAM_NAME) == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
        robot_parameters.wheel_radius = new_param->value.integer_value; //! This local variable should probably be better handled, seems abit redundant
        xQueueOverwrite(robot_params_queue, &robot_parameters);
        if (save_int_to_nvs(robot_parameters.wheel_radius, ROBOT_WHEEL_RADIUS_PARAM_NAME) == ESP_OK) {
            ESP_LOGI(MROS_LOGGER_TAG, "Parameter for wheel radius changed to %ld mm", robot_parameters.wheel_radius);
            return true;
        } else {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to save wheel radius to NVS");
            return false;
        }
    }

    if (strcmp(new_param->name.data, ROBOT_TRACK_WIDTH_PARAM_NAME) == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
        robot_parameters.track_width = new_param->value.integer_value;
        xQueueOverwrite(robot_params_queue, &robot_parameters);
        if (save_int_to_nvs(robot_parameters.track_width, ROBOT_TRACK_WIDTH_PARAM_NAME) == ESP_OK) {
            ESP_LOGI(MROS_LOGGER_TAG, "Parameter for track width changed to %ld mm", robot_parameters.track_width);
            return true;
        } else {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to save track width to NVS");
            return false;
        }
    }

    if (strcmp(new_param->name.data, ROBOT_WHEEL_BASE_PARAM_NAME) == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
        robot_parameters.wheel_base = new_param->value.integer_value;
        xQueueOverwrite(robot_params_queue, &robot_parameters);
        if (save_int_to_nvs(robot_parameters.wheel_base, ROBOT_WHEEL_BASE_PARAM_NAME) == ESP_OK) {
            ESP_LOGI(MROS_LOGGER_TAG, "Parameter for wheel base changed to %ld mm", robot_parameters.wheel_base);
            return true;
        } else {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to save wheel base to NVS");
            return false;
        }
    }

    if (strcmp(new_param->name.data, MAX_ANG_VEL_PARAM_NAME) == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
        robot_parameters.max_angular_velocity = (float)new_param->value.integer_value / 1000.0f; // Convert back to float
        xQueueOverwrite(robot_params_queue, &robot_parameters);
        if (save_int_to_nvs((int32_t)(robot_parameters.max_angular_velocity * 1000), MAX_ANG_VEL_PARAM_NAME) == ESP_OK) {
            ESP_LOGI(MROS_LOGGER_TAG, "Parameter for max angular velocity changed to %.3f", robot_parameters.max_angular_velocity);
            return true;
        } else {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to save max angular velocity to NVS");
            return false;
        }
    }

    if (strcmp(new_param->name.data, CORR_WEIGHT_PARAM_NAME) == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
        robot_parameters.correction_weight = (float)new_param->value.integer_value / 1000.0f; // Convert back to float
        xQueueOverwrite(robot_params_queue, &robot_parameters);
        if (save_int_to_nvs((int32_t)(robot_parameters.correction_weight * 1000), CORR_WEIGHT_PARAM_NAME) == ESP_OK) {
            ESP_LOGI(MROS_LOGGER_TAG, "Parameter for correction weight changed to %.3f", robot_parameters.correction_weight);
            return true;
        } else {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to save correction weight to NVS");
            return false;
        }
    }

    if (strcmp(new_param->name.data, DEADBEAT_START_PARAM_NAME) == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
        robot_parameters.deadbeat_start = (float)new_param->value.integer_value / 1000.0f; // Convert back to float
        xQueueOverwrite(robot_params_queue, &robot_parameters);
        if (save_int_to_nvs((int32_t)(robot_parameters.deadbeat_start * 1000), DEADBEAT_START_PARAM_NAME) == ESP_OK) {
            ESP_LOGI(MROS_LOGGER_TAG, "Parameter for deadbeat start changed to %.3f", robot_parameters.deadbeat_start);
            return true;
        } else {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to save deadbeat start to NVS");
            return false;
        }
    }

    if (strcmp(new_param->name.data, DEADBEAT_END_PARAM_NAME) == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
        robot_parameters.deadbeat_end = (float)new_param->value.integer_value / 1000.0f; // Convert back to float
        xQueueOverwrite(robot_params_queue, &robot_parameters);
        if (save_int_to_nvs((int32_t)(robot_parameters.deadbeat_end * 1000), DEADBEAT_END_PARAM_NAME) == ESP_OK) {
            ESP_LOGI(MROS_LOGGER_TAG, "Parameter for deadbeat end changed to %.3f", robot_parameters.deadbeat_end);
            return true;
        } else {
            ESP_LOGE(MROS_LOGGER_TAG, "Failed to save deadbeat end to NVS");
            return false;
        }
    }

    ESP_LOGW(MROS_LOGGER_TAG, "Unknown parameter %s", new_param->name.data);
    return false;
}

esp_err_t robot_parameters_get(robot_parameters_t *params) {
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueuePeek(robot_params_queue, params, NO_WAIT) != pdTRUE) {
        ESP_LOGE(MROS_LOGGER_TAG, "Failed to peek robot parameters queue");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t robot_parameters_get_preconfigured(robot_parameters_t *params) {
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }
    params->wheel_radius = ROBOT_WHEEL_RADIUS_MM;
    params->track_width = ROBOT_TRACK_WIDTH_MM;
    params->wheel_base = ROBOT_WHEEL_BASE_MM;
    params->correction_weight = (float)CORR_WEIGHT / 1000.0f;
    params->max_angular_velocity = (float)MAX_ANG_VEL / 1000.0f;
    params->deadbeat_start = (float)DEADBEAT_START / 1000.0f;
    params->deadbeat_end = (float)DEADBEAT_END / 1000.0f;
    params->kp = (float)KP / 1000.0f;
    return ESP_OK;
}