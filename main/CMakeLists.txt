idf_component_register(SRCS "pc_watchdog.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_timer esp_wifi esp_http_client esp-tls nvs_flash json lwip)