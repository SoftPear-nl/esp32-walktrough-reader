idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash esp_lcd driver spiffs nvs_flash
                       INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ../flash_fs FLASH_IN_PROJECT)
