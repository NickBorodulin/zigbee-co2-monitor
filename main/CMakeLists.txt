idf_component_register(
        SRCS "zigbee_CO2.c"
        INCLUDE_DIRS "."
        REQUIRES
            esp_driver_i2c
            esp_driver_ledc
            esp_adc
            esp_lcd
            scd4x
            espressif__esp-zigbee-lib
            nvs_flash
            app_update
        PRIV_REQUIRES
            spiffs
)

# Create and flash the SPIFFS image alongside the application image.
spiffs_create_partition_image(display_fs display_fs FLASH_IN_PROJECT)
