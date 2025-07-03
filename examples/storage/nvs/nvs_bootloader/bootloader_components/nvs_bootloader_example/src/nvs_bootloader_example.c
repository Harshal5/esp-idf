/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "nvs_bootloader.h"
#include "nvs_sec_provider.h"
#include "led_strip.h"

#define NVS_PART_LABEL       "nvs"
#define NVS_PART_NAMESPACE   "device_state"

static const char* TAG = "nvs_bootloader_example";

// Function used to tell the linker to include this file
// with all its symbols.
//
void bootloader_hooks_include(void){
}

// not used in this example
void bootloader_before_init(void) {
}

#define BLINK_GPIO 8
static uint8_t s_led_state = 0;
static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));

    led_strip_clear(led_strip);
}

// function hook called at the end of the bootloader standard code
// this is the 'main' function of the example
void bootloader_after_init(void) {

#if CONFIG_NVS_ENCRYPTION
    nvs_sec_cfg_t cfg = {};
    nvs_sec_scheme_t *sec_scheme_handle = NULL;
#if CONFIG_NVS_SEC_KEY_PROTECT_USING_HMAC
    nvs_sec_config_hmac_t sec_scheme_cfg = NVS_SEC_PROVIDER_CFG_HMAC_DEFAULT();
    if (nvs_sec_provider_register_hmac(&sec_scheme_cfg, &sec_scheme_handle) != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "Registering the HMAC scheme failed");
        return;
    }
#endif /* CONFIG_NVS_SEC_KEY_PROTECT_USING_HMAC */

    if (nvs_bootloader_read_security_cfg(sec_scheme_handle, &cfg) != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "Reading the NVS security configuration failed");
        return;
    }

    if (nvs_bootloader_secure_init(&cfg) != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "Secure initialization of NVS failed");
        return;
    }

    nvs_bootloader_read_list_t device_state_info[] = {
        { .namespace_name = "device_state",  .key_name = "switch",  .value_type = NVS_TYPE_U8 },
    };

    esp_err_t ret = nvs_bootloader_read(NVS_PART_LABEL, sizeof(device_state_info) / sizeof(nvs_bootloader_read_list_t), device_state_info);
    if (ret != ESP_OK) {
        ESP_EARLY_LOGE(TAG, "Reading the NVS partition failed");
        return;
    }

    s_led_state = device_state_info[0].value.u8_val;

    nvs_bootloader_secure_deinit();
#endif /* CONFIG_NVS_ENCRYPTION */

    configure_led();
    blink_led();
    ESP_EARLY_LOGI(TAG, "Device state restored: LED %s", s_led_state == 1 ? "ON" : "OFF");
}
