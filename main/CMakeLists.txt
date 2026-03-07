idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash esp_lcd driver esp_timer
                       INCLUDE_DIRS ".")
