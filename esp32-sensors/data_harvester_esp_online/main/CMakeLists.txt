idf_component_register(SRCS "harvester_sensors.c"
                    INCLUDE_DIRS ""
                    REQUIRES sht3x bh1750 scd4x esp_wifi nvs_flash json)
