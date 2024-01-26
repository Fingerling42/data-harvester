#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Include header files of sensors
#include "sht3x.h"

// Tag of executable for logging
static const char *TAG = "harvester_sensors";

// Devices descriptors
static sht3x_t sht3x_dev;

// I2C pin numbers
#define I2C_MASTER_SCL_IO           19     // GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO           18     // GPIO number used for I2C master data
#define I2C_MASTER_BUS              0      // I2C master i2c port number

// Devices address
#define SHT3X_ADDR					0x44


void app_main(void)
{
	// Initialize I2C
	ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize SHT3x sensor
    float temperature;
    float humidity;

    memset(&sht3x_dev, 0, sizeof(sht3x_t));

    ESP_ERROR_CHECK(sht3x_init_desc(&sht3x_dev, SHT3X_ADDR, I2C_MASTER_BUS, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(sht3x_init(&sht3x_dev));

    ESP_LOGI(TAG, "Sensors initialized successfully");

    TickType_t last_wakeup = xTaskGetTickCount();

    // Main loop
    while(true)
    {
        // Perform one measurement of SHT3x sensor
        ESP_ERROR_CHECK(sht3x_measure(&sht3x_dev, &temperature, &humidity));
        printf("sht3x_temp: %.4f\n", temperature);
        printf("sht3x_humi: %.4f\n", humidity);

        // Wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(5000));
    }
}
