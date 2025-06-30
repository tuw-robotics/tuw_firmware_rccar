#ifndef MROS_TRANSPORT_H
#define MROS_TRANSPORT_H

#include <uxr/client/transport.h>

/**
 * @brief Custom transport for serial communication
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @return true if the transport is successfully opened,
 * @return false if the transport could not be opened
 */
bool serial_open(struct uxrCustomTransport *transport);

/**
 * @brief Closes the custom serial transport
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @return true if the transport is successfully closed,
 * @return false if the transport could not be closed
 */
bool serial_close(struct uxrCustomTransport *transport);

/**
 * @brief Sets the custom serial transport to non-blocking mode
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @return true if the transport is successfully set to non-blocking mode,
 * @return false if the transport could not be set to non-blocking mode
 */
size_t serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);

/**
 * @brief Reads data from the custom serial transport
 *
 * @param transport Pointer to the uxrCustomTransport structure
 * @param buf Buffer to store the read data
 * @param len Length of the buffer
 * @param timeout Timeout for the read operation in milliseconds
 * @param err Pointer to store any error code
 * @return size_t Number of bytes read, or 0 on timeout or error
 */
size_t serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

#endif // MROS_TRANSPORT_H