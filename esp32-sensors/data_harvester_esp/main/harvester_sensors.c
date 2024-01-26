#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "sht3x.h"

static const char *TAG = "harvester_sensors";

#define I2C_MASTER_SCL_IO           19      					/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           18      				   	/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_BUS              0                          	/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          	/*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          	/*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

/**
 * i2c initialization
 */
static esp_err_t i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_BUS;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /**
     * main loop
     */
    while(1)
    {
    	ESP_LOGI(TAG, "Continue working...");
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
