#include <stdio.h>
#include <inttypes.h>
#include <sys/socket.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_crc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_bridge.h"
#include "esp_mesh_lite.h"

static const char *TAG = "MESH_TIMING_GATE";

// I2C Configuration
#define I2C_MASTER_SCL_IO           GPIO_NUM_14
#define I2C_MASTER_SDA_IO           GPIO_NUM_21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define TF_ADDR                     0x10
#define RTC_ADDR                    0b1101000
#define RTC_REG_SECONDS             0x00

// Timing Gate Configuration
#define THRESHOLD                   50
#define DEBOUNCE_TIME              5
#define DETECT                     5

// UART Configuration
#define UART_PORT                  UART_NUM_1
#define UART_BUF_SIZE              1024
#define UART_TX_PIN                GPIO_NUM_21
#define UART_RX_PIN                GPIO_NUM_14

// RTC SQW Pin
#define SQW_GPIO                   GPIO_NUM_12

#define LED                        GPIO_NUM_19

// TCP Server Configuration
static int g_sockfd = -1;

// Timing Gate State
static int state = 0;
static long change_time = 0;
static long last_activation = 0;

// RTC Sync Variables
static volatile int64_t last_sync_us = 0;
static volatile int64_t rtc_seconds = 0;
static portMUX_TYPE rtc_sync_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t uart_event_queue = NULL;

// Mesh packet structure
typedef struct {
    uint32_t seq_num;
    uint16_t crc;
    uint16_t len;
    uint8_t data[200];
} mesh_packet_t;

// Helper Functions
long get_time_ms() {
    return (long)(esp_timer_get_time() / 1000);
}

static uint8_t bcd2dec(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0F);
}

static uint8_t dec2bcd(uint8_t val) {
    return ((val / 10) << 4) + (val % 10);
}

// Debounce function
int debounce(int input) {
    long current_time = get_time_ms();
    if (input != state) {
        if (change_time == 0) {
            change_time = current_time;
        } else if (current_time - change_time >= DEBOUNCE_TIME) {
            state = input;
            change_time = 0;
            if (input == 1) {
                last_activation = current_time;
            }
        }
    } else {
        change_time = 0;
    }
    return state;
}

void tf_set_config(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_NUM_1, UART_BUF_SIZE * 2, 0, 20, &uart_event_queue, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t packet[] = {0x5A, 0x05, 0x0A, 0x01, 0};
    uint8_t checksum = 0;
    // Calculate checksum (simple sum of all bytes)
    for (int i = 0; i < sizeof(packet); i++) {
        checksum += packet[i];
    }
    packet[4] = checksum;
    // Send the data bytes
    uart_write_bytes(UART_PORT, (const char*)packet, sizeof(packet));
    ESP_LOGI("UART", "Sent packet: 0x%02X 0x%02X 0x%02X 0x%02X, Checksum: 0x%02X",
             packet[0], packet[1], packet[2], packet[3], checksum);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint8_t save[] = {0x5A, 0x04, 0x11, 0x6F};
    uart_write_bytes(UART_PORT, (const char *)save, sizeof(save));

    uart_driver_delete(UART_NUM_1);
}

// TFMini LiDAR Functions
uint16_t getTfData() {
    uint8_t getData[] = {0x5A, 0x05, 0x00, 0x01, 0x60};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TF_ADDR << 1) | I2C_MASTER_WRITE, true);
    for(int i = 0; i < sizeof(getData); i++) {
        i2c_master_write_byte(cmd, getData[i], false);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        uint8_t read_data[9];
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TF_ADDR << 1) | I2C_MASTER_READ, true);
        for(int i = 0; i < 8; i++) {
            i2c_master_read_byte(cmd, &read_data[i], I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, &read_data[8], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            return read_data[3] << 8 | read_data[2];
        } else {
            ESP_LOGE(TAG, "Failed to read LiDAR response");
            return 0;
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command to LiDAR");
        return 0;
    }

    return 0;
}

// DS3231 RTC Functions
static esp_err_t ds3231_get_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RTC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, RTC_REG_SECONDS, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RTC_ADDR << 1) | I2C_MASTER_READ, true);

    uint8_t data[3];
    i2c_master_read(cmd, data, 2, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    *seconds = bcd2dec(data[0]);
    *minutes = bcd2dec(data[1]);
    *hours   = bcd2dec(data[2] & 0x3F);

    return ESP_OK;
}

esp_err_t ds3231_set_time(struct tm *time) {
    uint8_t data[7];

    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);
    data[3] = dec2bcd(time->tm_wday + 1);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_mon + 1);
    data[6] = dec2bcd(time->tm_year - 100);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RTC_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, RTC_REG_SECONDS, true);
    i2c_master_write(cmd, data, 7, true);

    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

// SQW Interrupt Handler
static void IRAM_ATTR sqw_handler(void* arg) {
    uint8_t h, m, s;
    if (ds3231_get_time(&h, &m, &s) == ESP_OK) {
        portENTER_CRITICAL_ISR(&rtc_sync_mux);
        rtc_seconds = h * 3600 + m * 60 + s;
        last_sync_us = esp_timer_get_time();
        portEXIT_CRITICAL_ISR(&rtc_sync_mux);
    }
}

// Get synced microseconds
int64_t get_synced_micros(void) {
    portENTER_CRITICAL(&rtc_sync_mux);
    int64_t base_rtc_seconds = rtc_seconds;
    int64_t last_sync = last_sync_us;
    portEXIT_CRITICAL(&rtc_sync_mux);

    int64_t now_us = esp_timer_get_time();
    int64_t delta_us = now_us - last_sync;
    return base_rtc_seconds * 1000000 + delta_us;
}


// UART Initialization
static void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, 0, 20, &uart_event_queue, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART initialized successfully");
}

// TCP Client Functions
static int socket_tcp_client_create(const char *ip, uint16_t port) {
    ESP_LOGD(TAG, "Create a tcp client, ip: %s, port: %d", ip, port);

    int sockfd = -1;
    struct ifreq iface;
    memset(&iface, 0x0, sizeof(iface));
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = inet_addr(ip),
    };

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        ESP_LOGE(TAG, "socket create, sockfd: %d", sockfd);
        goto ERR_EXIT;
    }

    esp_netif_get_netif_impl_name(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), iface.ifr_name);
    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(struct ifreq)) != 0) {
        ESP_LOGE(TAG, "Bind [sock=%d] to interface %s fail", sockfd, iface.ifr_name);
    }

    esp_err_t ret = connect(sockfd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
        ESP_LOGD(TAG, "socket connect, ret: %d, ip: %s, port: %d", ret, ip, port);
        goto ERR_EXIT;
    }
    return sockfd;

ERR_EXIT:
    if (sockfd != -1) {
        close(sockfd);
    }
    return -1;
}

// Timing Gate Task
void timing_gate_task(void *arg) {
    uint16_t dist = 0;
    uint16_t prevDist = 0;
    mesh_packet_t packet;
    int counter = 0;
    bool currentState = false;
    bool prev = false;
    float epoch = 0;
    float diff = 0;

    // 90 Hz loop timing (11.11 ms period)
    const TickType_t loop_period = pdMS_TO_TICKS(11);
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "Timing gate task started");

    // Wait a bit for I2C to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    while(1) {
        // Try to reconnect if disconnected, but don't block
        if (g_sockfd == -1) {
            g_sockfd = socket_tcp_client_create("192.168.71.1", 8080);
            if (g_sockfd != -1) {
                ESP_LOGI(TAG, "TCP connected successfully");
            }
        }

        prevDist = dist;
        dist = getTfData();

        prev = currentState;
        currentState = debounce(dist < DETECT);

        if (!prev && currentState) {
            packet.seq_num = counter;
            float current = (get_synced_micros() / 1e6) - (float)DEBOUNCE_TIME / 1e3;
            diff = current - epoch;
            epoch = current;

            int data_len = snprintf((char*)packet.data, sizeof(packet.data),
                                   "Lap Timer sent time: %f", diff);
            packet.len = data_len;

            packet.crc = 0;
            packet.crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)&packet, sizeof(mesh_packet_t));

            ESP_LOGI(TAG, "*** DETECTED! *** #%d, lap_time: %.3f sec, dist: %d", counter, diff, dist);

            if (g_sockfd != -1) {
                char *tcp_data;
                uint8_t sta_mac[6];
                esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

                size_t size = asprintf(&tcp_data,
                    "{\"src_addr\": \"" MACSTR "\",\"seq\": %d,\"lap_time\": %.3f,\"level\": %d}\r\n",
                    MAC2STR(sta_mac), counter, diff, esp_mesh_lite_get_level());

                int write_result = write(g_sockfd, tcp_data, size);
                if (write_result <= 0) {
                    ESP_LOGE(TAG, "TCP write failed: %d, errno: %d", write_result, errno);
                    close(g_sockfd);
                    g_sockfd = -1;
                    // Blink rapidly 3 times to show TCP failed (non-blocking pattern)
                    for (int i = 0; i < 6; i++) {
                        gpio_set_level(LED, i % 2);
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                } else {
                    ESP_LOGI(TAG, "TCP sent: %d bytes", write_result);
                    // Keep LED on for 300ms to show success
                    vTaskDelay(pdMS_TO_TICKS(300));
                }
                free(tcp_data);
            } else {
                ESP_LOGW(TAG, "No TCP connection");
                // Quick blink pattern for no connection
                for (int i = 0; i < 6; i++) {
                    gpio_set_level(LED, i % 2);
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
            }

            gpio_set_level(LED, 0);
            counter++;
        }

        vTaskDelayUntil(&last_wake_time, loop_period);
    }
}

// WiFi Disconnect Event Handler
static void wifi_event_sta_disconnected_handler(void *arg, esp_event_base_t event_base,
                                               int32_t event_id, void *event_data) {
    wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW(TAG, "WiFi disconnected, reason: %d", event->reason);

    // Close TCP socket on disconnect
    if (g_sockfd != -1) {
        close(g_sockfd);
        g_sockfd = -1;
        ESP_LOGI(TAG, "TCP socket closed due to WiFi disconnect");
    }
}

void led_task() {
    for (;;) {
        if (g_sockfd == -1) {
            gpio_set_level(LED, 1);
        } else {
            gpio_set_level(LED, 0);
        }

        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

// IP Event Handler
static void ip_event_sta_got_ip_handler(void *arg, esp_event_base_t event_base,
                                        int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

    // Close old socket if exists and force reconnect
    if (g_sockfd != -1) {
        close(g_sockfd);
        g_sockfd = -1;
    }
}

// WiFi Initialization
static void wifi_init(void) {
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "myssid",
            .password = "mypassword",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .threshold.rssi = -127,
            .pmf_cfg = {
                .capable = false,
                .required = false
            },
            .rm_enabled = 0,
            .btm_enabled = 0,
            .mbo_enabled = 0,
            .ft_enabled = 0,
            .owe_enabled = 0,
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);

    wifi_config_t wifi_softap_config = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 4,
            .pmf_cfg = {
                .capable = false,
                .required = false,
            },
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_softap_config);
}

void app_wifi_set_softap_info(void) {
    char softap_ssid[33];
    char softap_psw[64];
    uint8_t softap_mac[6];
    size_t ssid_size = sizeof(softap_ssid);
    size_t psw_size = sizeof(softap_psw);

    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));
    memset(softap_psw, 0x0, sizeof(softap_psw));

    if (esp_mesh_lite_get_softap_ssid_from_nvs(softap_ssid, &ssid_size) == ESP_OK) {
        ESP_LOGI(TAG, "Get ssid from nvs: %s", softap_ssid);
    } else {
#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
        snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x",
                CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
        snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
        ESP_LOGI(TAG, "Get ssid from nvs failed, set ssid: %s", softap_ssid);
    }

    if (esp_mesh_lite_get_softap_psw_from_nvs(softap_psw, &psw_size) == ESP_OK) {
        ESP_LOGI(TAG, "Get psw from nvs: [HIDDEN]");
    } else {
        strlcpy(softap_psw, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(softap_psw));
        ESP_LOGI(TAG, "Get psw from nvs failed, set psw: [HIDDEN]");
    }

    esp_mesh_lite_set_softap_info(softap_ssid, softap_psw);
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();

    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();
    esp_mesh_lite_init(&mesh_lite_config);

    app_wifi_set_softap_info();
    esp_mesh_lite_start();

    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // tf_set_config();

    // Initialize UART for RTC sync
    uart_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Waiting for rtc sync");

    // Read timestamp from UART
    uint8_t data[128];
    int total_len = 0;
    int64_t start_time = esp_timer_get_time();
    const int64_t timeout_us = 10 * 1e6;

    while (total_len < sizeof(data) - 1) {
        int len = uart_read_bytes(UART_NUM_0, &data[total_len], 1, 100 / portTICK_PERIOD_MS);

        if (len > 0) {
            total_len += len;
            if (data[total_len - 1] == '\n' || data[total_len - 1] == '\r' || data[total_len - 1] == '\0') {
                break;
            }
        }

        if (esp_timer_get_time() - start_time > timeout_us) {
            ESP_LOGI(TAG, "No timestamp received");
            break;
        }
    }

    if (total_len >= 6) {
        data[total_len] = '\0';
        char hour_str[3] = {data[0], data[1], '\0'};
        char min_str[3] = {data[2], data[3], '\0'};
        char sec_str[3] = {data[4], data[5], '\0'};

        int hours = atoi(hour_str);
        int minutes = atoi(min_str);
        int seconds = atoi(sec_str);

        if (hours >= 0 && hours <= 23 && minutes >= 0 && minutes <= 59 && seconds >= 0 && seconds <= 59) {
            struct tm time;
            time.tm_hour = hours;
            time.tm_min = minutes;
            time.tm_sec = seconds;
            time.tm_wday = 0;
            time.tm_mday = 1;
            time.tm_mon = 0;
            time.tm_year = 125;

            ret = ds3231_set_time(&time);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Time set successfully to %02d:%02d:%02d", hours, minutes, seconds);
            } else {
                ESP_LOGE(TAG, "Failed to set time");
            }
        }
    }

    uart_driver_delete(UART_NUM_0);

    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LED,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&led_conf);

    // Setup SQW interrupt
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << SQW_GPIO,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SQW_GPIO, sqw_handler, NULL);

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &ip_event_sta_got_ip_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                                        &wifi_event_sta_disconnected_handler, NULL, NULL));

    // Start timing gate task - runs continuously regardless of WiFi
    ESP_LOGI(TAG, "Starting timing gate task...");
    xTaskCreate(timing_gate_task, "timing_gate_task", 4 * 1024, NULL, 6, NULL);

    xTaskCreate(led_task, "led_task", 1024, NULL, 7, NULL);
}
