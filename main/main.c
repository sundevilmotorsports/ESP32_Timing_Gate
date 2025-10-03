#include <stdio.h>
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mesh.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <ds3231.h>
#include <string.h>
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "espnow.h"
#include "time.h"

static const char *TAG = "MAIN";

#define I2C_MASTER_SCL_IO           GPIO_NUM_14                 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21                 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       10000
#define TF_ADDR                     0x10
#define RTC_ADDR                    0b1101000
#define THRESHOLD 50

#define RTC_REG_SECONDS 0x00

#define DEBOUNCE_TIME 5
#define DETECT 50

#define UART_PORT UART_NUM_0
#define UART_BUF_SIZE 1024
#define RD_BUF_SIZE 128

#define SQW_GPIO GPIO_NUM_12
static int state = 0;
static long change_time = 0;
static long last_activation = 0;

static volatile int64_t last_sync_us = 0;   // Last sync timestamp (µs)
static volatile int64_t rtc_seconds = 0;    // DS3231 absolute time in seconds

static QueueHandle_t uart_event_queue = NULL;

long get_time_ms() { return clock() * 1000 / CLOCKS_PER_SEC; }

int debounce(int input) {
  long current_time = get_time_ms();  
  if (input != state) {
    if (change_time == 0) {
      change_time = current_time;
    } else if (current_time - change_time >= DEBOUNCE_TIME) {
      state = input;
      change_time = 0;      if (input == 1) {
        last_activation = current_time;
      }
    }
  } else {
    change_time = 0;
  }  return state;
}

i2c_master_dev_handle_t tfmini_dev_handle;
i2c_master_bus_handle_t bus_handle;

// Function Declarations
uint16_t getTfData(){
    uint8_t getData[] = {0x5A, 0x05, 0x00, 0x01, 0x60};    // Send command to device
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TF_ADDR << 1) | I2C_MASTER_WRITE, true);
    for(int i = 0; i< sizeof(getData); i++){
        i2c_master_write_byte(cmd, getData[i], false);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);    
    if (ret == ESP_OK) {
        // Read 9 bytes response from device
        uint8_t read_data[9];
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TF_ADDR << 1) | I2C_MASTER_READ, true);
        for(int i = 0; i < 8; i++){
            i2c_master_read_byte(cmd, &read_data[i], I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, &read_data[8], I2C_MASTER_NACK); // Last byte with NACK
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);        
        if (ret == ESP_OK) { //  && (read_data[3] << 8 | read_data[2]) < 500
            // ESP_LOGI("I2C", "LiDAR Response:");
            // printf("Distance %u cm\n", read_data[3] << 8 | read_data[2]);
            return read_data[3] << 8 | read_data[2];
        } else {
            ESP_LOGE("I2C", "Failed to read response");
            return 0;
        }
    }    
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to send command to device at address 0x10");
        return 0;
    }    
    return 0;
}

// --- Helper: Read BCD from DS3231 ---
static uint8_t bcd2dec(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0F);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

// --- Read seconds from DS3231 ---
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

// --- Interrupt Handler on 1khz SQW rising edge ---
static void IRAM_ATTR sqw_handler(void* arg) {
    uint8_t h, m, s;
    if (ds3231_get_time(&h, &m, &s) == ESP_OK) {
        rtc_seconds = h * 3600 + m * 60 + s;
        last_sync_us = esp_timer_get_time();
    }
}

// --- Get synced microseconds ---
int64_t get_synced_micros(void) {
    int64_t now_us = esp_timer_get_time();
    int64_t delta_us = now_us - last_sync_us;
    return rtc_seconds * 1000000 + delta_us;
}

esp_err_t ds3231_set_time(struct tm *time) {
    uint8_t data[7];

    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);

    /* The week data must be in the range 1 to 7, and to keep the start on the
     * same day as for tm_wday have it start at 1 on Sunday. */
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

static void uart_init(void){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 20, &uart_event_queue, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART initialized successfully");
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    wifi_init();
    espnow_init();

    uint8_t buf[7];
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

    uart_init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Waiting for rtc sync");

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

    if (total_len > 0) {
        data[total_len] = '\0';
        // printf("Received: %.*s\n", total_len, data);

        if (total_len >= 6) {
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
                // Not needed
                time.tm_wday = 0;
                time.tm_mday = 0;
                time.tm_mon = 0;
                time.tm_year = 125;

                esp_err_t ret = ds3231_set_time(&time);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Time set successfully to %02d:%02d:%02d", hours, minutes, seconds);
                } else {
                    ESP_LOGE(TAG, "Failed to set time");
                }
            } else {
                ESP_LOGE(TAG, "Invalid time values: %02d:%02d:%02d", hours, minutes, seconds);
            }
        } else {
            ESP_LOGE(TAG, "Invalid timestamp format. Expected HHMMSS, got %d characters", total_len);
        }
    } else {
        ESP_LOGI(TAG, "No timestamp received");
    }

    uart_driver_delete(UART_NUM_0);

    uint16_t dist = 0;
    uint16_t prevDist = 0;    // Send command to device
    espnow_data_t packet;
    int counter = 0;
    bool currentState = false;
    bool prev = false;
    float epoch = 0;
    float diff = 0;

    
    // SQW input pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << SQW_GPIO,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SQW_GPIO, sqw_handler, NULL);
    
    // 90 Hz loop timing (11.11 ms period)
    const TickType_t loop_period = pdMS_TO_TICKS(11);  // ~11.11 ms for 90 Hz
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while(1) {
        prevDist = dist;
        dist = getTfData(); 
        // currentState = (abs(dist - prevDist) > THRESHOLD) ? 1 : 0;
        // if (currentState) {
        //     detect = !detect;
        // }
        prev = currentState;
        currentState = debounce(dist < DETECT);
        printf("Time Delta: %f\n", (float)(get_synced_micros() / 1e6) - (float)(esp_timer_get_time() / 1e6));

        if (!prev && currentState) {
            packet.seq_num = counter;
            float current = (get_synced_micros() / 1e6) - (float)DEBOUNCE_TIME / 1e3;  // seconds
            diff = current - epoch;
            epoch = current;
            
            snprintf(packet.text, sizeof(packet.text), "Lap Timer sent time: %f", diff);

            packet.crc = 0;
            packet.crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)&packet, sizeof(espnow_data_t));
            
            ESP_LOGI(TAG, "Sending message #%d...", counter);
            espnow_send_once(&packet);
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
        
        // Maintain 90 Hz loop rate
        vTaskDelayUntil(&last_wake_time, loop_period);
    }

}
