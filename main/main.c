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

static int state = 0;
static long change_time = 0;
static long last_activation = 0;

long get_time_ms() { return clock() * 1000 / CLOCKS_PER_SEC; }

int debounce(int input) {
  long current_time = get_time_ms();  
  if (input != state) {
    if (change_time == 0) {
      change_time = current_time;
    } else if (current_time - change_time >= 500) {
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
uint16_t getTfData();

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
    // esp_restart();
    uint16_t dist = 0;
    uint16_t prevDist = 0;    // Send command to device
    espnow_data_t packet;
    int counter = 0;
    int currentState = -1;
    bool detect = false;
    float epoch = 0;
    float diff = 0;
    while(1) {
        prevDist = dist;
        dist = getTfData(); 
        currentState = (abs(dist - prevDist) > THRESHOLD) ? 1 : 0;
        if (currentState) {
            detect = !detect;
        }

        if (debounce(dist < 50)) {
            packet.seq_num = counter;
            float current = esp_timer_get_time() / 1e6;  // seconds
            diff = current - epoch;
            epoch = current;
            
            snprintf(packet.text, sizeof(packet.text), "Lap Timer sent time: %f", diff);

            packet.crc = 0;
            packet.crc = esp_crc16_le(UINT16_MAX, (uint8_t const *)&packet, sizeof(espnow_data_t));
            
            ESP_LOGI(TAG, "Sending message #%d...", counter);
            espnow_send_once(&packet);
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
    }

}

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