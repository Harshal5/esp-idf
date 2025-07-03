idf_component_register(SRCS "bootloader_hooks_example_main.c" "app_utilities.c"
                    REQUIRES nvs_flash nvs_sec_provider esp_wifi wifi_provisioning esp-tls
                    INCLUDE_DIRS ".")

if(NOT CONFIG_NVS_ENCRYPTION)
    nvs_create_partition_image(nvs ../nvs_data.csv FLASH_IN_PROJECT)
else()
    if(CONFIG_NVS_SEC_KEY_PROTECT_USING_FLASH_ENC)
        esptool_py_flash_to_partition(flash "nvs_key" ${PROJECT_DIR}/main/encryption_keys.bin)
        esptool_py_flash_to_partition(flash "nvs" ${PROJECT_DIR}/main/nvs_encrypted.bin)
    else()  # NVS Encryption using HMAC (CONFIG_NVS_SEC_KEY_PROTECT_USING_HMAC)
        esptool_py_flash_to_partition(flash "nvs" ${PROJECT_DIR}/main/nvs_encrypted_hmac.bin)
    endif()
endif()
