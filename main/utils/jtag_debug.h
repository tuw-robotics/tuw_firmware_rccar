#ifndef JTAG_DEBUG_H
#define JTAG_DEBUG_H

#include <esp_err.h>

#ifdef CONFIG_JTAG_BUFF_SIZE
#define JTAG_BUFF_SIZE CONFIG_JTAG_BUFF_SIZE
#else
#define JTAG_BUFF_SIZE 1024
#endif

/**
 * @brief Initializes the JTAG debug interface
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t jtag_debug_init(void);

#endif // JTAG_DEBUG_H