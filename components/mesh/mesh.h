#ifndef MESH_H
#define MESH_H

#include "esp_err.h"

typedef struct {
    uint16_t seq_num;
    uint8_t data[30];
    uint16_t crc;
    size_t len;
} mesh_packet_t;

/**
 * @brief Initializes and starts the ESP-MESH network.
 *
 * This function handles the entire setup process for the mesh network,
 * including initializing Wi-Fi, event handlers, and mesh configurations.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t mesh_init(void);
// Use this to send timestamp to root node
void mesh_send_once(const mesh_packet_t *packet);

#endif
