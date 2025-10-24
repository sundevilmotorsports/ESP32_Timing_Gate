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
        snprintf((char*)packet_data->data, PAYLOAD_BUFFER_SIZE, "Hello Broadcast! Msg %lu", message_count);

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
}

static void espnow_deinit(espnow_send_param_t *send_param) {
    if (send_param != NULL) {
        free(send_param->buffer);
        free(send_param);
    }
    esp_now_deinit();
}