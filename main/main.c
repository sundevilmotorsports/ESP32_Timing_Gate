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

static const char *TAG = "MAIN";

#define HWSERIAL Serial1
#define CENTIMETERS 500
#define UPDATE_TIMEOUT 3000 //miliseconds
#define SEND_TIMEOUT 500 // milliseconds
#define LED_TIMEOUT 100 // milliseconds

#define GPIO_PIN 2

#define TESTING_LIDAR true
#define TESTING_RTC true
#define LED 9

int starting_millis = 0;
uint8_t starting_year = 0;
uint8_t starting_month = 0;
uint8_t starting_day = 0;
uint8_t starting_hour = 0;
uint8_t starting_minute = 0;
uint8_t starting_second = 0;

bool radio = false;
bool century;
bool h12Flag;
bool pmFlag;

#define I2C_MASTER_SCL_IO           GPIO_NUM_14                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       10000

static const char *TFMINI_TAG = "TFMINI_I2C";
static const char *DS3231_TAG = "ds3231";

int distance = 0;
int strength = 0;
int start = 0;
int prevSend = 0;
int prevLED = 0;
bool waiting = false;

uint8_t general[16];

i2c_master_dev_handle_t tfmini_dev_handle;
i2c_master_bus_handle_t bus_handle;

// Function Declarations
static esp_err_t i2c_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void getTFminiData(int* distance, int* strength);
void loop(void *pvParameters);
void setup();

// i2c_dev_t ds3231_dev;
// i2c_dev_t tfmini_dev;

void app_main(void)
{
    //RUN WIFI LIBRARY AS COMPONENT
    // Initialize NVS
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_LOGI(TAG, "Initializing Mesh network...");
    // ESP_ERROR_CHECK(mesh_init());
    
    // ESP_ERROR_CHECK(i2cdev_init());
    // ESP_ERROR_CHECK(ds3231_init_desc(&ds3231_dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    i2c_master_init(&bus_handle, &tfmini_dev_handle);

    // Manually initialize the TFMini device descriptor
    // tfmini_dev.port = I2C_MASTER_NUM;
    // tfmini_dev.addr = 0x10; // TFMini I2C address
    // tfmini_dev.cfg.sda_io_num = I2C_MASTER_SDA_IO;
    // tfmini_dev.cfg.scl_io_num = I2C_MASTER_SCL_IO;
    // tfmini_dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;

    // gpio_config_t io_conf = {
    //     .pin_bit_mask = (1ULL << GPIO_PIN),
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type = GPIO_INTR_DISABLE
    // };

    // gpio_config(&io_conf);

    ESP_LOGI(TAG, "I2C initialized successfully");

    // setup();

    xTaskCreate(loop, "main_loop", 4192, NULL, 5, NULL);
}

static esp_err_t i2c_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t i2c_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

void add_i2c_device(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, int device_address) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1, // using any available port
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
    add_i2c_device(bus_handle, dev_handle, 0x10); //TFMINI
    add_i2c_device(bus_handle, dev_handle, 0b1101000); //D23231
}

void getTFminiData(int* distance, int* strength) {
    uint8_t command[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
    uint8_t rx_data[9];
    esp_err_t err;

    err = i2c_master_transmit(tfmini_dev_handle, command, sizeof(command), I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TFMINI_TAG, "Failed to send measurement command: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(30)); 

    err = i2c_master_receive(tfmini_dev_handle, rx_data, sizeof(rx_data), I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TFMINI_TAG, "Failed to read data from TFMini: %s", esp_err_to_name(err));
        return;
    }

    if (rx_data[0] == 0x59 && rx_data[1] == 0x59) {
        int checksum = 0;
        for (int i = 0; i < 8; i++) {
            checksum += rx_data[i];
        }
        
        if (rx_data[8] == (checksum & 0xFF)) {
            *distance = rx_data[2] | (rx_data[3] << 8);
            *strength = rx_data[4] | (rx_data[5] << 8);
        } else {
            ESP_LOGW(TFMINI_TAG, "TFMini checksum error");
        }
    } else {
        ESP_LOGW(TFMINI_TAG, "Invalid TFMini header received");
    }
}

void loop(void *pvParameters) {
    while (1) {
        bool sent;
        getTFminiData(&distance, &strength);
        if (distance != 65535) {
            if (distance < CENTIMETERS && (esp_timer_get_time() / 1000) - prevLED > LED_TIMEOUT) {
                // gpio_set_level(GPIO_PIN, 0);
            } else {
                // gpio_set_level(GPIO_PIN, 1);
            }
            if (!waiting && distance < CENTIMETERS) {
                start = (esp_timer_get_time() / 1000);
                waiting = true;
                sent = false;

                general[0] = (uint8_t)1; // Timing Gate ID
                general[1] = (uint8_t)0; // gate number (increment with each new gate)

                general[2] = starting_second;
                general[3] = starting_minute;
                general[4] = starting_hour;
                general[5] = starting_day;
                general[6] = starting_month;
                general[7] = starting_year;

                general[8] = (starting_millis >> 24) & 0xff;
                general[9] = (starting_millis >> 16) & 0xff;
                general[10] = (starting_millis >> 8) & 0xff;
                general[11] = starting_millis & 0xff;

                general[12] = (start >> 24) & 0xff;
                general[13] = (start >> 16) & 0xff;
                general[14] = (start >> 8) & 0xff;
                general[15] = start & 0xff;

            } else if (waiting && (esp_timer_get_time() / 1000) - start > UPDATE_TIMEOUT) {
                waiting = false;
                ESP_LOGI(DS3231_TAG, "ended timeout");
            }
            // SEND TO ROOT
        }
        if (TESTING_LIDAR) {
            printf("%dcm\tstrength: %d\tstart: %d\n", distance, strength, start);
        }
        if (TESTING_RTC) {
            printf("Starting Time Test:\t%d/%d/%d %d:%d:%d\n", starting_month, starting_day, starting_year, starting_hour, starting_minute, starting_second);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// void setup() {
//     ESP_LOGI("SETUP", "\n" __FILE__ " " __DATE__ " " __TIME__);
    
//     // gpio_set_level(GPIO_NUM_13, 1);
//     time_t seconds=time(NULL);
//     struct tm* current_time=localtime(&seconds);
//     ds3231_set_time(&ds3231_dev, current_time);
//     ESP_LOGI("SETUP", "UPDATED RTC");
    
//     struct tm now;
//     memset(&now, 0, sizeof(struct tm));
//     ds3231_get_time(&ds3231_dev, &now);

//     // int start_sec = now.tm_sec;
//     // while(start_sec == myRTC.getSecond());
    
//     starting_millis = esp_timer_get_time() / 1000;
//     starting_year = now.tm_year;
//     starting_month = now.tm_mon;
//     starting_day = now.tm_mday;
//     starting_hour = now.tm_hour;
//     starting_minute = now.tm_min;
//     starting_second = now.tm_sec;
// }