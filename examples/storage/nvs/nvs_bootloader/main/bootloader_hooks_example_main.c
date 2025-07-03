/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "nvs_flash.h"
#include "nvs_sec_provider.h"

#define NVS_PART_LABEL       "nvs"
#define NVS_PART_NAMESPACE   "device_state"

static const char *TAG = "example";

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

void app_main(void)
{

    /**
     * 1. Fetch the LED state from NVS
     * 2. Configure the LED
     * 3. Play with the LED state
     * 4. Save the LED state to NVS
     */

    // 1. Fetch the LED state from NVS
    nvs_sec_cfg_t cfg = {};
    nvs_sec_scheme_t *sec_scheme_handle = nvs_flash_get_default_security_scheme();

    esp_err_t ret = nvs_flash_read_security_cfg_v2(sec_scheme_handle, &cfg);
    if (ret != ESP_OK) {
        /* We shall not generate keys here as that must have been done in default NVS partition initialization case */
        ESP_LOGE(TAG, "Failed to read NVS security cfg: [0x%02X] (%s)", ret, esp_err_to_name(ret));
    }

    ret = nvs_flash_secure_init_partition(NVS_PART_LABEL, &cfg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "NVS partition \"%s\" is encrypted.", NVS_PART_LABEL);
    }

    nvs_handle_t my_handle;
    ret = nvs_open_from_partition(NVS_PART_LABEL, NVS_PART_NAMESPACE, NVS_READWRITE, &my_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS partition: [0x%02X] (%s)", ret, esp_err_to_name(ret));
    }

    ret = nvs_get_u8(my_handle, "switch", &s_led_state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get switch state: [0x%02X] (%s)", ret, esp_err_to_name(ret));
    }

    // 2. Configure the LED
    configure_led();
    blink_led();
    ESP_LOGI(TAG, "Device state restored: LED %s", s_led_state == 1 ? "ON" : "OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // 3. Play with the LED state
    for (int i = 0; i < 4; i++) {
        s_led_state = !s_led_state;
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == 1 ? "ON" : "OFF");
        blink_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // 4. Save the LED state to NVS
    ret = nvs_set_u8(my_handle, "switch", s_led_state);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set switch state: [0x%02X] (%s)", ret, esp_err_to_name(ret));
    }

    ret = nvs_commit(my_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS partition: [0x%02X] (%s)", ret, esp_err_to_name(ret));
    }

    nvs_close(my_handle);

    // 5. Power off the LED just to make sure the next idf.py monitor doesn't show the LED on by default
    // and the bootloader does try enabling the LED
    s_led_state = 0;
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == 1 ? "ON" : "OFF");
    blink_led();
}
