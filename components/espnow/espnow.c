// /* ESPNOW Example

//    This example code is in the Public Domain (or CC0 licensed, at your option.)

//    Unless required by applicable law or agreed to in writing, this
//    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
//    CONDITIONS OF ANY KIND, either express or implied.
// */

// /*
//    This example shows how to use ESPNOW.
//    Prepare two device, one for sending ESPNOW data and another for receiving
//    ESPNOW data.
// */
// #include <stdlib.h>
// #include <time.h>
// #include <string.h>
// #include <assert.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include "freertos/timers.h"
// #include "nvs_flash.h"
// #include "esp_random.h"
// #include "esp_event.h"
// #include "esp_netif.h"
// #include "esp_wifi.h"
// #include "esp_log.h"
// #include "esp_mac.h"
// #include "esp_now.h"
// #include "esp_crc.h"
// #include "espnow.h"
// #include <stdio.h>
// #include <stdint.h>
// #include <ctype.h>

// #define ESPNOW_MAXDELAY 512

// static const char *TAG = "espnow_example";

// static QueueHandle_t s_example_espnow_queue = NULL;

// static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
// static uint16_t s_example_espnow_seq[EXAMPLE_ESPNOW_DATA_MAX] = { 0, 0 };

// static void example_espnow_deinit(example_espnow_send_param_t *send_param);

// /* WiFi should start before using ESPNOW */
// void example_wifi_init(void)
// {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
//     ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
//     ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
//     ESP_ERROR_CHECK( esp_wifi_start());
//     ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

// #if CONFIG_ESPNOW_ENABLE_LONG_RANGE
//     ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
// #endif
// }

// /* ESPNOW sending or receiving callback function is called in WiFi task.
//  * Users should not do lengthy operations from this task. Instead, post
//  * necessary data to a queue and handle it from a lower priority task. */
// static void example_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
// {
//     example_espnow_event_t evt;
//     example_espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

//     if (tx_info == NULL) {
//         ESP_LOGE(TAG, "Send cb arg error");
//         return;
//     }

//     evt.id = EXAMPLE_ESPNOW_SEND_CB;
//     memcpy(send_cb->mac_addr, tx_info->des_addr, ESP_NOW_ETH_ALEN);
//     send_cb->status = status;
//     if (xQueueSend(s_example_espnow_queue, &evt, ESPNOW_MAXDELAY) != pdTRUE) {
//         ESP_LOGW(TAG, "Send send queue fail");
//     }
// }

// /* Prepare ESPNOW data to be sent. */
// void example_espnow_data_prepare(example_espnow_send_param_t *send_param)
// {
//     example_espnow_data_t *buf = (example_espnow_data_t *)send_param->buffer;

//     assert(send_param->len >= sizeof(example_espnow_data_t));

//     buf->type = IS_BROADCAST_ADDR(send_param->dest_mac) ? EXAMPLE_ESPNOW_DATA_BROADCAST : EXAMPLE_ESPNOW_DATA_UNICAST;
//     buf->state = send_param->state;
//     buf->seq_num = s_example_espnow_seq[buf->type]++;
//     buf->crc = 0;
//     buf->magic = send_param->magic;
//     /* Fill all remaining bytes after the data with random values */
//     esp_fill_random(buf->payload, send_param->len - sizeof(example_espnow_data_t));
//     buf->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)buf, send_param->len);
// }

// static void espnow_send_timestamp(void *pvParameters)
// {
//     example_espnow_event_t evt;
//     int count = 0;

//     vTaskDelay(5000 / portTICK_PERIOD_MS);
//     ESP_LOGI(TAG, "Start sending broadcast data");

//     example_espnow_send_param_t *send_param = pvParameters;

//     // while (xQueueReceive(s_example_espnow_queue, &evt, portMAX_DELAY) == pdTRUE) {
//     while(1) {
//         if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
//             ESP_LOGE(TAG, "Send error");
//             example_espnow_deinit(send_param);
//             vTaskDelete(NULL);
//         }
//         ESP_LOGI("ESPNOW SEND", "Sent %d", count);
//         count++;
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }

// esp_err_t example_espnow_init(void)
// {
//     example_espnow_send_param_t *send_param;

//     s_example_espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(example_espnow_event_t));
//     if (s_example_espnow_queue == NULL) {
//         ESP_LOGE(TAG, "Create queue fail");
//         return ESP_FAIL;
//     }

//     /* Initialize ESPNOW and register sending and receiving callback function. */
//     ESP_ERROR_CHECK( esp_now_init() );
//     ESP_ERROR_CHECK( esp_now_register_send_cb(example_espnow_send_cb) );
// #if CONFIG_ESPNOW_ENABLE_POWER_SAVE
//     ESP_ERROR_CHECK( esp_now_set_wake_window(CONFIG_ESPNOW_WAKE_WINDOW) );
//     ESP_ERROR_CHECK( esp_wifi_connectionless_module_set_wake_interval(CONFIG_ESPNOW_WAKE_INTERVAL) );
// #endif
//     /* Set primary master key. */
//     ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

//     /* Add broadcast peer information to peer list. */
//     esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
//     if (peer == NULL) {
//         ESP_LOGE(TAG, "Malloc peer information fail");
//         vQueueDelete(s_example_espnow_queue);
//         s_example_espnow_queue = NULL;
//         esp_now_deinit();
//         return ESP_FAIL;
//     }
//     memset(peer, 0, sizeof(esp_now_peer_info_t));
//     peer->channel = CONFIG_ESPNOW_CHANNEL;
//     peer->ifidx = ESPNOW_WIFI_IF;
//     peer->encrypt = false;
//     memcpy(peer->peer_addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
//     ESP_ERROR_CHECK( esp_now_add_peer(peer) );
//     free(peer);

//     uint8_t rx_buffer[] = "spain";
//     send_param = create_params(rx_buffer);
//     xTaskCreate(espnow_send_timestamp, "espnow_send_timestamp", 2048, send_param, 4, NULL);

//     return ESP_OK;
// }

// example_espnow_send_param_t* create_params(uint8_t *buffer) {
//     example_espnow_send_param_t *send_param;
//     /* Initialize sending parameters. */
//     send_param = malloc(sizeof(example_espnow_send_param_t));
//     // if (send_param == NULL) {
//     //     ESP_LOGE(TAG, "Malloc send parameter fail");
//     //     vQueueDelete(s_example_espnow_queue);
//     //     s_example_espnow_queue = NULL;
//     //     esp_now_deinit();
//     //     return ESP_FAIL;
//     // }
//     memset(send_param, 0, sizeof(example_espnow_send_param_t));
//     send_param->unicast = false;
//     send_param->broadcast = true;
//     send_param->state = 0;
//     send_param->magic = esp_random();
//     send_param->count = CONFIG_ESPNOW_SEND_COUNT;
//     send_param->delay = CONFIG_ESPNOW_SEND_DELAY;
//     send_param->len = CONFIG_ESPNOW_SEND_LEN;
//     send_param->buffer = buffer;
//     // if (send_param->buffer == NULL) {
//     //     ESP_LOGE(TAG, "Malloc send buffer fail");
//     //     free(send_param);
//     //     vQueueDelete(s_example_espnow_queue);
//     //     s_example_espnow_queue = NULL;
//     //     esp_now_deinit();
//     //     return ESP_FAIL;
//     // }
//     memcpy(send_param->dest_mac, s_example_broadcast_mac, ESP_NOW_ETH_ALEN);
//     example_espnow_data_prepare(send_param);
//     return send_param;
// }

// static void example_espnow_deinit(example_espnow_send_param_t *send_param)
// {
//     free(send_param->buffer);
//     free(send_param);
//     vQueueDelete(s_example_espnow_queue);
//     s_example_espnow_queue = NULL;
//     esp_now_deinit();
// }

// void func(void)
// {
//     // Initialize NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK( nvs_flash_erase() );
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK( ret );

//     example_wifi_init();
//     example_espnow_init();
// }

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "espnow.h"

// --- Configuration ---
#define ESPNOW_WIFI_MODE    WIFI_MODE_STA
#define ESPNOW_WIFI_IF      ESP_IF_WIFI_STA
#define ESPNOW_CHANNEL      1
#define SEND_PERIOD_MS      500 // Send a message every 500 milliseconds
#define PAYLOAD_BUFFER_SIZE 50  // Max size of the data payload

// --- Globals & TAG ---
static const char *TAG = "ESPNOW_SENDER";
static QueueHandle_t s_espnow_queue;

// Broadcast MAC address
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void espnow_deinit(espnow_send_param_t *send_param);

void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    if (tx_info == NULL || tx_info->des_addr == NULL) {
        ESP_LOGE(TAG, "Send CB error: Argument is NULL");
        return;
    }

    // The MAC address is inside the tx_info struct in this version
    const uint8_t *mac_addr = tx_info->des_addr;

    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "Send to " MACSTR " successful", MAC2STR(mac_addr));
    } else {
        ESP_LOGW(TAG, "Send to " MACSTR " failed, status: %d", MAC2STR(mac_addr), status);
    }
}

esp_err_t espnow_send_once(const espnow_data_t *data_to_send) {
    if (data_to_send == NULL) {
        ESP_LOGE(TAG, "Data to send is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send the data
    esp_err_t result = esp_now_send(s_broadcast_mac, (const uint8_t *)data_to_send, sizeof(espnow_data_t));
    
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW single send error: %s", esp_err_to_name(result));
    }
    
    return result;
}

static void espnow_send_task(void *pvParameters) {
    espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameters;
    espnow_data_t *packet_data = (espnow_data_t *)send_param->buffer;
    uint32_t message_count = 0;

    ESP_LOGI(TAG, "Starting periodic sending task...");

    while (1) {
        // 1. Prepare the data payload
        packet_data->seq_num = message_count++;
        snprintf(packet_data->text, PAYLOAD_BUFFER_SIZE, "Hello Broadcast! Msg %lu", message_count);

        // 2. Calculate and set the CRC checksum
        // The CRC is calculated over the packet data, excluding the CRC field itself.
        packet_data->crc = 0;
        packet_data->crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)packet_data, send_param->len);

        // 3. Send the data
        if (esp_now_send(send_param->dest_mac, send_param->buffer, send_param->len) != ESP_OK) {
            ESP_LOGE(TAG, "ESP-NOW send error");
            // In a real application, you might want to handle this error more gracefully,
            // e.g., by retrying a few times before giving up.
        }

        // 4. Wait for the next sending period
        vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD_MS));
    }

    // This part of the code will not be reached in this example,
    // but it's good practice to include cleanup logic.
    ESP_LOGI(TAG, "Stopping sending task and cleaning up.");
    espnow_deinit(send_param);
    vTaskDelete(NULL);
}

void espnow_init(void) {
    // 1. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // 2. Register the send callback function
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // 3. Add a broadcast peer
    esp_now_peer_info_t peer_info = {};
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.ifidx = ESPNOW_WIFI_IF;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    // 4. Prepare send parameters and create the sending task
    // IMPORTANT: We must dynamically allocate memory for send_param and its buffer
    // because it's passed to a task that will outlive this function.
    espnow_send_param_t *send_param = malloc(sizeof(espnow_send_param_t));
    if (send_param == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for send parameters");
        esp_now_deinit();
        return;
    }
    memset(send_param, 0, sizeof(espnow_send_param_t));

    send_param->len = sizeof(espnow_data_t);
    send_param->buffer = malloc(send_param->len);
    if (send_param->buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for send buffer");
        free(send_param);
        esp_now_deinit();
        return;
    }

    memcpy(send_param->dest_mac, s_broadcast_mac, ESP_NOW_ETH_ALEN);

    // xTaskCreate(espnow_send_task, "espnow_send_task", 2048, send_param, 4, NULL);
    // int message_count = 0;
    // for (int i = 0; i < 5; i++) {
    //     // 1. Create and populate the data packet
    //     espnow_data_t packet;
    //     packet.seq_num = message_count++;
    //     snprintf(packet.text, sizeof(packet), "Hello Broadcast! Msg #%d", message_count);

    //     // 2. Calculate and set the CRC checksum
    //     packet.crc = 0; // Set to 0 before calculating
    //     packet.crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)&packet, sizeof(espnow_data_t));
        
    //     // 3. Call the singular send function
    //     ESP_LOGI(TAG, "Sending message #%lu...", message_count);
    //     espnow_send_once(&packet);
        
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    // }
}

static void espnow_deinit(espnow_send_param_t *send_param) {
    if (send_param != NULL) {
        free(send_param->buffer);
        free(send_param);
    }
    esp_now_deinit();
}

// /**
//  * @brief Main application entry point.
//  */
// void app_main(void) {
//     // Initialize NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // Initialize networking stack
//     wifi_init();

//     // Initialize ESP-NOW
//     espnow_init();
// }