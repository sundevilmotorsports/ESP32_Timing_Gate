/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_H
#define ESPNOW_H

#include "esp_now.h"
#include "esp_crc.h"

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef struct __attribute__((packed)) {
    uint16_t seq_num;   // Sequence number to keep track of messages
    char text[30];         // Message content
    uint16_t crc;       // CRC checksum for data integrity
} espnow_data_t;


typedef struct {
    uint8_t *buffer;                        // Buffer containing the packet to send
    size_t len;                             // Length of the packet
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];     // Destination MAC address
} espnow_send_param_t;


void espnow_init(void);
void wifi_init(void);
esp_err_t espnow_send_once(const espnow_data_t *data_to_send);

#endif