idf_component_register(SRCS "app_main.c" "time_sync.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi nvs_flash wifi_provisioning esp-tls esp_psram mbedtls esp_timer
                       INCLUDE_DIRS ".")
