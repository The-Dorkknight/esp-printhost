idf_component_register(SRCS "main.c" "settings.c" "led.c" "link.cpp" "storage.c"
                       INCLUDE_DIRS "."
                       REQUIRES host esp_wifi esp_event esp_netif esp_http_server nvs_flash driver esp_timer
                                spi_flash esp_rom dns_server)
