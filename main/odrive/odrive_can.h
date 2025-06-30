
#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Pack ODrive CAN message ID
 *
 * @param node_id Configured ODrive node ID (0-31)
 * @param cmd_id Command ID (0-31)
 * @return uint32_t Packed CAN message ID
 */
static inline uint32_t odrive_can_pack_id(uint32_t node_id, uint32_t cmd_id) { return (node_id << 5) | (cmd_id & 0x1F); }

/**
 * @brief Unpack ODrive CAN message ID
 *
 * @param msg_id Packed CAN message ID
 * @return uint32_t Node ID (0-31)
 */
static inline uint32_t odrive_can_unpack_node(uint32_t msg_id) { return msg_id >> 5; }

/**
 * @brief Unpack ODrive CAN command ID
 *
 * @param msg_id Packed CAN message ID
 * @return uint32_t Command ID (0-31)
 */
static inline uint32_t odrive_can_unpack_cmd(uint32_t msg_id) { return msg_id & 0x1F; }

#ifdef __cplusplus
}
#endif

#endif // ODRIVE_CAN_H
