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
#include <time.h>
#include "esp_sleep.h"

#define WATER_PIN GPIO_NUM_2
#define SENSOR_PIN ADC1_CHANNEL_4
static const adc_atten_t atten = ADC_ATTEN_DB_11;
lcd1602_t lcdtmp = {0};
int watertime = 1;
static const char *TAG = "Watering_System";

void LCD_init(void) {
    lcd1602_i2c_init(21, 22, 0);

    lcdtmp.i2caddr = 0x3F;
    lcdtmp.backlight = 1;

    lcd1602_dcb_set(&lcdtmp, 1, 0, 0);

    lcd1602_init(&lcdtmp);
}

void BMP280_init(void) {
    esp_err_t err;
    uint8_t id = 0;

    err = bmp280_i2c_reset();
    if (err != BMP280_OK) {
        ESP_LOGE(TAG, "Error setting up BMP280 device!");
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
    } else {
        ESP_LOGE(TAG, "BMP280 initialization failed!");
    }
}
void setup(){
    esp_rom_gpio_pad_select_gpio(WATER_PIN);
    gpio_set_direction(WATER_PIN, GPIO_MODE_OUTPUT);

}
void measure(void) {
    float moisture;
    bmp280_data_t bmp280_data;

    // Read moisture from ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_PIN, atten);

    uint32_t sensor_analog = adc1_get_raw(SENSOR_PIN);
    moisture = 100 - (float)sensor_analog / 4095.0 * 100.0;

    char moisture_str[20];
    sprintf(moisture_str, "Moisture = %.2f%%", (float)moisture);

    lcd1602_set_pos(&lcdtmp, 0, 0);
    lcd1602_send_string(&lcdtmp, moisture_str);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Moisture: %.2f%%", (float)moisture);

    // Read data from BMP280
    if (bmp280_i2c_read_data(&bmp280_data) == BMP280_OK) {
        ESP_LOGI(TAG, "Temperature: %.01f °C", (float)bmp280_data.temperature / 100);
        
        // Convert temperature to string
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "Temp: %.01f C", (float)bmp280_data.temperature / 100);

        // Update LCD display
        lcd1602_set_pos(&lcdtmp, 1, 0);
        lcd1602_send_string(&lcdtmp, temp_str);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } else {
        ESP_LOGE(TAG, "Error reading data from BMP280!");
    }

    // Control watering based on moisture level
    if (moisture < 60 && (float)bmp280_data.temperature/100 > 10 && (float)bmp280_data.temperature/100 < 40 ) {
        gpio_set_level(WATER_PIN, 1); // Turn on water pump
        vTaskDelay(watertime *   
        
        7000 / portTICK_PERIOD_MS);

    }
    else
    {
        gpio_set_level(WATER_PIN, 0); // Turn off water pump
        vTaskDelay(watertime * 1000 / portTICK_PERIOD_MS); // 
    }
    
}

void app_main(void) {
    
    setup();
    LCD_init();
    BMP280_init();
    while (1) {
            measure();
        const unsigned int sleep_duration = 3600;
        ESP_LOGI(TAG, "Entering deep sleep for %d seconds", sleep_duration);
        esp_deep_sleep(sleep_duration * 1000000); // Convert seconds to microseconds
    }
}
