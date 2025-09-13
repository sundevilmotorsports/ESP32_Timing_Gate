#ifndef MESH_H
#define MESH_H

#include "esp_err.h"

/**
 * @brief Initializes and starts the ESP-MESH network.
 *
 * This function handles the entire setup process for the mesh network,
 * including initializing Wi-Fi, event handlers, and mesh configurations.
 *
 * @return esp_err_t ESP_OK on success, otherwise an error code.
 */
esp_err_t mesh_init(void);

#endif
