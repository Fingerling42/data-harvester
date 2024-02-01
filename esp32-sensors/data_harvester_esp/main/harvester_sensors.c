#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"

// Include header files of sensors
#include "sht3x.h"
#include "bh1750.h"
#include "scd4x.h"

// Tag of executable for logging
static const char *TAG = "harvester_sensors";

// FreeRTOS event group to signal when wi-fi is connected
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// Devices descriptors
static sht3x_t sht3x_dev;
static i2c_dev_t bh1750_dev;
static i2c_dev_t scd4x_dev = { 0 };

// Bits for event group: connected to the AP with an IP or failed to connect after the maximum amount of retries
#define WIFI_CONNECTED_BIT 			BIT0
#define WIFI_FAIL_BIT      			BIT1

// Wi-Fi connection parameters
#define WIFI_SSID      						""
#define WIFI_PASS      						""
#define MAXIMUM_RETRY  						5
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD	WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SAE_MODE					WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER				""

// SNTP server
#define SNTP_TIME_SERVER					"pool.ntp.org"


// I2C pin numbers
#define I2C_MASTER_SCL_IO           19     // GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO           18     // GPIO number used for I2C master data
#define I2C_MASTER_BUS              0      // I2C master i2c port number

// Devices address
#define SHT3X_ADDR					0x44
#define BH1750_ADDR					BH1750_ADDR_LO

#define DELAY_TICKS					5000 	// Sensors readings delay in millisec

static void obtain_time(void)
{

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(SNTP_TIME_SERVER);
    esp_netif_sntp_init(&config);

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 15;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    esp_netif_sntp_deinit();
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "Wi-Fi initialization is finished.");

    // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler()
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "Unexpected event");
    }
}


void app_main(void)
{
	struct timeval tv_now;
	double timestamp;

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	// Connect to Wi-Fi access point
	wifi_init();

	// Initialize time for NTP sync
	time_t now;
	obtain_time();
    time(&now);

	// Set timezone
    setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
    tzset();


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

    scd4x_init_desc(&scd4x_dev, I2C_MASTER_BUS, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    scd4x_wake_up(&scd4x_dev);
    vTaskDelay(pdMS_TO_TICKS(30));
    scd4x_stop_periodic_measurement(&scd4x_dev);
    vTaskDelay(pdMS_TO_TICKS(500));
    scd4x_reinit(&scd4x_dev);

    uint16_t serial[3];
    scd4x_get_serial_number(&scd4x_dev, serial, serial + 1, serial + 2);
    ESP_LOGI(TAG, "SCD4x serial number: 0x%04x%04x%04x", serial[0], serial[1], serial[2]);

    scd4x_start_periodic_measurement(&scd4x_dev);

    vTaskDelay(pdMS_TO_TICKS(DELAY_TICKS));
    ESP_LOGI(TAG, "Sensors initialized successfully");

    TickType_t last_wakeup = xTaskGetTickCount();

    // Main loop
    while(true)
    {
    	// Get timestamp
    	gettimeofday(&tv_now, NULL);
    	timestamp = tv_now.tv_sec + tv_now.tv_usec * 0.000001;

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

        // Wait until delay is over
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(DELAY_TICKS));

    }
}
