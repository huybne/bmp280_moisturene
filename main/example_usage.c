// #include "esp_log.h"
// #include "esp_err.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/adc.h"
// #include "driver/gpio.h"
// #include "driver/rtc_io.h"
// #include "esp_sleep.h"

// // BMP280 components
// #include "bmp280_i2c.h"
// #include "bmp280_i2c_hal.h"

// // LCD 1602 I2C components
// #include "lcd_i2c.h"

// static const char *TAG = "example_usage";

// #define MEASURE_INTERVAL (5 * 60 * 60 * 1000) // Đo mỗi 5 giờ (5 giờ = 5 * 60 phút * 60 giây * 1000 mili giây)

// typedef struct {
//     lcd1602_t lcd;
// } LCD1602;

// void LCD1602_init(LCD1602 *lcd, uint8_t i2caddr, int sdaPin, int sclPin) {
//     lcd->lcd.i2caddr = i2caddr;
//     lcd->lcd.backlight = 1;
//     lcd1602_i2c_init(21, 22, 0);
//     lcd1602_dcb_set(&(lcd->lcd), 1, 0, 0);
//     lcd1602_init(&(lcd->lcd));
// }

// void LCD1602_setCursor(LCD1602 *lcd, int col, int row) {
//     lcd1602_set_pos(&(lcd->lcd), col, row);
// }

// void LCD1602_print(LCD1602 *lcd, const char* message) {
//     lcd1602_send_string(&(lcd->lcd), message);
// }

// typedef struct {
//     bmp280_data_t bmp280Data;
// } BMP280;

// void BMP280_init(BMP280 *bmp280) {
//     bmp280_i2c_hal_init();

//     esp_err_t err = bmp280_i2c_reset();
//     if (err != BMP280_OK) {
//         ESP_LOGE(TAG, "Error setting the device!");
//     }

//     err += bmp280_i2c_set_calib();
//     ESP_LOGI(TAG, "Calibration data setting: %s", err == BMP280_OK ? "Successful" : "Failed");

//     err += bmp280_i2c_write_power_mode(POWERMODE_NORMAL);
//     ESP_LOGI(TAG, "Setting to normal mode: %s", err == BMP280_OK ? "Successful" : "Failed");

//     err += bmp280_i2c_write_config_filter(FILTER_4);
//     bmp280_ctrl_meas_t ctrl_meas = {
//         .osrs_press = OSRS_x4,
//         .osrs_tmp = OSRS_x1,
//     };
//     err += bmp280_i2c_write_osrs(ctrl_meas);

//     if (err == BMP280_OK) {
//         ESP_LOGI(TAG, "BMP280 initialization successful");
//     } else {
//         ESP_LOGE(TAG, "BMP280 initialization failed!");
//     }
// }

// float BMP280_readTemperature(BMP280 *bmp280) {
//     if (bmp280_i2c_read_data(&(bmp280->bmp280Data)) == BMP280_OK) {
//         return (float)(bmp280->bmp280Data.temperature) / 100;
//     } else {
//         ESP_LOGE(TAG, "Error reading BMP280 data!");
//         return 0.0f;
//     }
// }

// void app_main(void)
// {
//     // Initialize LCD object
//     LCD1602 lcd;
//     LCD1602_init(&lcd, 0x3F, 21, 22);

//     // Initialize BMP280 object
//     BMP280 bmp280;
//     BMP280_init(&bmp280);

//     // Configure ADC
//     adc1_config_width(ADC_WIDTH_BIT_12);
//     adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

//     while (1) {
//         // Read temperature from BMP280
//         float temperature = BMP280_readTemperature(&bmp280);
//         ESP_LOGI(TAG, "Temperature: %.01f °C", temperature);

//         // Read moisture from ADC
//         uint32_t sensor_analog = adc1_get_raw(ADC1_CHANNEL_4);
//         float moisture = 100 - (float)sensor_analog / 4095.0 * 100.0;

//         // Convert temperature and moisture to strings
//         char temp_str[16];
//         snprintf(temp_str, sizeof(temp_str), "Temp: %.01f C", temperature);

//         char moisture_str[20];
//         sprintf(moisture_str, "Moisture = %.2f%%", moisture);

//         // Update LCD display for temperature and moisture
//         LCD1602_setCursor(&lcd, 0, 0);
//         LCD1602_print(&lcd, temp_str);

//         LCD1602_setCursor(&lcd, 1, 1);
//         LCD1602_print(&lcd, moisture_str);

//         // Print temperature and moisture to the computer screen
//         ESP_LOGI(TAG, "Temperature: %.01f °C,Moisture: %.01f %%", temperature, moisture);

//         // Delay for a certain period
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }






#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "bmp280_i2c.h"
#include "bmp280_i2c_hal.h"

#define SENSOR_PIN ADC1_CHANNEL_4

static const adc_atten_t atten = ADC_ATTEN_DB_11;
lcd1602_t lcdtmp = {0};

static const char *TAG = "example_usage";

void setup(void) {
    lcd1602_i2c_init(21, 22, 0);

    lcdtmp.i2caddr = 0x3F;
    lcdtmp.backlight = 1;

    lcd1602_dcb_set(&lcdtmp, 1, 0, 0);

    lcd1602_init(&lcdtmp);
}

void loop(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_PIN, atten);

    uint32_t sensor_analog = adc1_get_raw(SENSOR_PIN);
    float moisture = (float)sensor_analog / 4095.0 * 100.0;

    char moisture_str[20];
    sprintf(moisture_str, "Moisture = %.2f%%", (float) moisture);

    lcd1602_set_pos(&lcdtmp, 0, 0);
    lcd1602_send_string(&lcdtmp, moisture_str);

    ESP_LOGI(TAG, "Moisture: %.2f%%", (float)moisture);

                // ESP_LOGI(TAG, "Temperature: %.01f °C", (float)bmp280_dt.temperature / 100);


    esp_err_t err;
    uint8_t id = 0;

    err = bmp280_i2c_reset();
    if (err != BMP280_OK) {
        ESP_LOGE(TAG, "Error setting the device!");
    }

    err += bmp280_i2c_read_part_number(&id);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Part number: 0x%02x", id);
    } else {
        ESP_LOGE(TAG, "Unable to read part number!");
    }

    err += bmp280_i2c_set_calib();
    ESP_LOGI(TAG, "Calibration data setting: %s", err == BMP280_OK ? "Successful" : "Failed");

    err += bmp280_i2c_write_power_mode(POWERMODE_NORMAL);
    ESP_LOGI(TAG, "Setting to normal mode: %s", err == BMP280_OK ? "Successful" : "Failed");

    err += bmp280_i2c_write_config_filter(FILTER_4);
    bmp280_ctrl_meas_t ctrl_meas = {
        .osrs_press = OSRS_x4,
        .osrs_tmp = OSRS_x1,
    };
    err += bmp280_i2c_write_osrs(ctrl_meas);

    if (err == BMP280_OK && id == 0x58) {
        ESP_LOGI(TAG, "BMP280 initialization successful");
        bmp280_data_t bmp280_dt;
        while (1) {
            // Reading data from BMP280
            if (bmp280_i2c_read_data(&bmp280_dt) == BMP280_OK) {
                ESP_LOGI(TAG, "Temperature: %.01f °C", (float)bmp280_dt.temperature / 100);

                // Convert temperature to string
                char temp_str[16];
                snprintf(temp_str, sizeof(temp_str), "Temp: %.01f C", (float)bmp280_dt.temperature / 100);

                // Update LCD display
                lcd1602_set_pos(&lcdtmp, 1, 0);
                lcd1602_send_string(&lcdtmp, temp_str);
            } else {
                ESP_LOGE(TAG, "Error reading data from BMP280!");
            }

            // Delay between measurements
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    } else {
        ESP_LOGE(TAG, "BMP280 initialization failed!");
    }
}

void app_main() {
    setup();
    while (1) {
        loop();
    }
}
 


 