#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Include header files of sensors
#include "sht3x.h"
#include "bh1750.h"
#include "scd4x.h"

// Tag of executable for logging
static const char *TAG = "harvester_sensors";

// Devices descriptors
static sht3x_t sht3x_dev;
static i2c_dev_t bh1750_dev;
// static i2c_dev_t scd4x_dev = { 0 };

// I2C pin numbers
#define I2C_MASTER_SCL_IO           19     // GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO           18     // GPIO number used for I2C master data
#define I2C_MASTER_BUS              0      // I2C master i2c port number

// Devices address
#define SHT3X_ADDR					0x44
#define BH1750_ADDR					BH1750_ADDR_LO


void app_main(void)
{
	struct timeval tv_now;
	float timestamp;

	// Initialize I2C
	ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize SHT3x sensor
    float temperature;
    float humidity;

    memset(&sht3x_dev, 0, sizeof(sht3x_t));

    ESP_ERROR_CHECK(sht3x_init_desc(&sht3x_dev, SHT3X_ADDR, I2C_MASTER_BUS, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(sht3x_init(&sht3x_dev));

    // Initialize BH1750 sensor
    uint16_t lux;

    memset(&bh1750_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(bh1750_init_desc(&bh1750_dev, BH1750_ADDR, I2C_MASTER_BUS, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(bh1750_setup(&bh1750_dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH));

    // Initialize SCD4x sensor
    uint16_t co2;
    float temp_scd4x, humi_scd4x;
    i2c_dev_t scd4x_dev = { 0 };

    scd4x_init_desc(&scd4x_dev, I2C_MASTER_BUS, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    scd4x_wake_up(&scd4x_dev);
    scd4x_stop_periodic_measurement(&scd4x_dev);
    scd4x_reinit(&scd4x_dev);

    uint16_t serial[3];
    scd4x_get_serial_number(&scd4x_dev, serial, serial + 1, serial + 2);
    ESP_LOGI(TAG, "SCD4x serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    scd4x_start_periodic_measurement(&scd4x_dev);

    ESP_LOGI(TAG, "Sensors initialized successfully");

    TickType_t last_wakeup = xTaskGetTickCount();

    // Main loop
    while(true)
    {
    	vTaskDelay(pdMS_TO_TICKS(5000));
    	gettimeofday(&tv_now, NULL);
    	timestamp = (float)tv_now.tv_sec + (float)tv_now.tv_usec * 0.000001;

        // Perform one measurement of SHT3x sensor
        ESP_ERROR_CHECK(sht3x_measure(&sht3x_dev, &temperature, &humidity));
        printf("[%.4f] sht3x_temp: %.4f\n", timestamp, temperature);
        printf("[%.4f] sht3x_humi: %.4f\n", timestamp, humidity);

        // Perform one measurement of BH1750 sensor
        if (bh1750_read(&bh1750_dev, &lux) != ESP_OK)
            printf("Could not read light sensor data\n");
        else
            printf("[%.4f] bh1750_lux: %d\n", timestamp, lux);

        // Perform one measurement of SCD4x sensor
        if (scd4x_read_measurement(&scd4x_dev, &co2, &temp_scd4x, &humi_scd4x) != ESP_OK)
        	printf("Could not read SCD4x sensor data\n");
        else
        	printf("[%.4f] scd4x_co2: %u\n", timestamp, co2);

        // Wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(5000));

    }
}
