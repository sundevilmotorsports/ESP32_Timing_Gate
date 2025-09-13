#include <stdio.h>
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mesh.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds3231.h>
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

#define TESTING_LIDAR false
#define TESTING_RTC false
#define LED 9

i2c_dev_t dev;

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

#define I2C_MASTER_SCL_IO           14                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       10000

#define ES3231_SENSOR_ADDR         0x68        /*!< Address of the ES3231 sensor */
#define ES3231_WRITE_REG_ADDR      0x75        /*!< Register addresses of the "who am I" register */
#define ES3231_RESET_BIT           7

static const char *TFMINI_TAG = "TFMINI_I2C";
static const char *DS3231_TAG = "ds3231";

i2c_master_dev_handle_t dev_handle;
i2c_master_bus_handle_t bus_handle;

int distance = 0;
int strength = 0;
int start = 0;
int prevSend = 0;
int prevLED = 0;
bool waiting = false;

uint8_t general[16];


// Function Declarations
static esp_err_t es3231_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);
void getTFminiData(int* distance, int* strength);
void ds3231_read(void *pvParameters);
void setup();
void ds3231(void *pvParameters);

void app_main(void)
{
    //RUN WIFI LIBRARY AS COMPONENT
    // Initialize NVS
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_LOGI(TAG, "Initializing Mesh network...");
    // ESP_ERROR_CHECK(mesh_init());
    
    ESP_ERROR_CHECK(i2cdev_init());
    memset(&dev, 0, sizeof(i2c_dev_t));
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 1, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
    // Instantiate I2C
    ESP_LOGI(TAG, "I2C initialized successfully");

    setup();

    xTaskCreate(ds3231_read, "ds3231_read", 4192, NULL, 5, NULL);
}

static esp_err_t es3231_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x1,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void getTFminiData(int* distance, int* strength) {
    static char i = 0;
    int j = 0;
    int checksum = 0; 
    uint8_t rx[9];
    ESP_ERROR_CHECK(es3231_register_read(dev_handle, 0x1, rx, 1));
    if (rx[0] != 0x59) {
        i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
        i = 0;
    } else if (i == 8) {
        for (j = 0; j < 8; j++) {
            checksum += rx[j];
        }
        if (rx[8] == (checksum % 256)) {
            *distance = rx[2] + rx[3] * 256;
            *strength = rx[4] + rx[5] * 256;
        }
    i = 0;
    } else {
        i++;
    }  
}

void ds3231_read(void *pvParameters) {
    bool sent;
    getTFminiData(&distance, &strength);
    if(distance != 65535){
        if(distance < CENTIMETERS && (esp_timer_get_time() / 1000) - prevLED > LED_TIMEOUT){
        gpio_set_level(GPIO_PIN, 0);
        } else {
        gpio_set_level(GPIO_PIN, 1);
        }
        if(!waiting && distance < CENTIMETERS){
        start = (esp_timer_get_time() / 1000);
        waiting = true;
        sent = false;

        general[0] = (uint8_t) 1; // Timing Gate ID
        general[1] = (uint8_t) 0; // gate number (increment with each new gate)

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

        } else if (waiting && (esp_timer_get_time() / 1000) - start > UPDATE_TIMEOUT){
        waiting = false;
        ESP_LOGI(DS3231_TAG, "ended timeout");
        }
        // SEND TO ROOT
    }
    if (TESTING_LIDAR){
        printf("%dcm\tstrength: %d\tstart: %d\n", distance, strength, start);
    }
    if (TESTING_RTC){
        printf("Starting Time Test:\t%d/%d/%d %d:%d:%d", starting_month, starting_day, starting_year, starting_hour, starting_minute, starting_second);
    }
}





void setup() {
    ESP_LOGI("SETUP", "\n" __FILE__ " " __DATE__ " " __TIME__);
    
    gpio_set_level(13, 1);
    time_t seconds=time(NULL);
    struct tm* current_time=localtime(&seconds);
    ds3231_set_time(&dev, current_time);
    ESP_LOGI("SETUP", "UPDATED RTC");
    
    struct tm* now;
    memset(&now, 0, sizeof(now)); 
    ds3231_get_time(&dev, now);
    // int start_sec = now->tm_sec;
    // while(start_sec == myRTC.getSecond()); cringle??
    
    // starting_millis = esp_timer_get_time() / 1000;
    // starting_year = now->tm_year;
    // starting_month = now->tm_mon;
    // starting_day = now->tm_mday;
    // starting_hour = now->tm_hour;
    // starting_minute = now->tm_min;
    // starting_second = now->tm_sec;
}