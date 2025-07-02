/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "sdkconfig.h"
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_rom_gpio.h"
#include "soc/spi_periph.h"
#include "driver/spi_master.h"
#include "esp_private/spi_common_internal.h"
#include "hal/spi_hal.h"
#include "hal/gpio_hal.h"
#include "new_spi_ll.h"

#define SPI_COMMON_ATTR IRAM_ATTR

static const char *SPI_TAG = "spi_common";

// GPIO HAL context for direct HAL layer access
static gpio_hal_context_t s_gpio_hal = {
    .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};

#define SPI_CHECK(a, str, ret_val, ...)  ESP_RETURN_ON_FALSE(a, ret_val, SPI_TAG, str, ##__VA_ARGS__)
#define SPI_CHECK_PIN(pin_num, pin_name, check_output) if (check_output) { \
            SPI_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(pin_num), pin_name" not valid", ESP_ERR_INVALID_ARG); \
        } else { \
            SPI_CHECK(GPIO_IS_VALID_GPIO(pin_num), pin_name" not valid", ESP_ERR_INVALID_ARG); \
        }
#define SPI_MAIN_BUS_DEFAULT() { \
        .host_id = 0, \
        .bus_attr = { \
            .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE, \
        }, \
    }

typedef struct {
    int host_id;
    spi_bus_attr_t bus_attr;
    // Removed dma_ctx - DMA not used in polling-only mode
} spicommon_bus_context_t;

static spicommon_bus_context_t s_mainbus = SPI_MAIN_BUS_DEFAULT();
static spicommon_bus_context_t* bus_ctx[SOC_SPI_PERIPH_NUM] = {&s_mainbus};

// Direct GPIO reservation implementation (replaces esp_gpio_reserve.h dependency)
// Uses same implementation as in esp_hw_support/esp_gpio_reserve.c
static uint64_t s_spi_reserved_pin_mask = ~(SOC_GPIO_VALID_GPIO_MASK);

//Periph 1 is 'claimed' by SPI flash code.
static bool spi_periph_claimed[SOC_SPI_PERIPH_NUM] = { true, false,
#if (SOC_SPI_PERIPH_NUM >= 3)
                                                       false,
#endif
#if (SOC_SPI_PERIPH_NUM >= 4)
                                                       false,
#endif
                                                     };

static const char* spi_claiming_func[3] = {NULL, NULL, NULL};

//----------------------------------------------------------alloc spi periph-------------------------------------------------------//
//Returns true if this peripheral is successfully claimed, false if otherwise.
bool SPI_COMMON_ATTR spicommon_periph_claim(spi_host_device_t host, const char* source)
{
    bool ret = false;
    if (!spi_periph_claimed[host]) {
        spi_periph_claimed[host] = true;
        ret = true;
    }
    if (ret) {
        spi_claiming_func[host] = source;
        _spi_ll_enable_bus_clock(host, true);
        _spi_ll_reset_register(host);
    } else {
        ESP_EARLY_LOGE(SPI_TAG, "SPI%d already claimed by %s.", host + 1, spi_claiming_func[host]);
    }
    return ret;
}

bool SPI_COMMON_ATTR spicommon_periph_in_use(spi_host_device_t host)
{
    return spi_periph_claimed[host];
}

//Returns true if this peripheral is successfully freed, false if otherwise.
bool SPI_COMMON_ATTR spicommon_periph_free(spi_host_device_t host)
{
    bool ret = false;
    if (spi_periph_claimed[host]) {
        spi_periph_claimed[host] = false;
        ret = true;
    }
    if (ret) {
        _spi_ll_enable_bus_clock(host, false);
    }
    return ret;
}

esp_err_t SPI_COMMON_ATTR spicommon_bus_alloc(spi_host_device_t host_id, const char *name)
{
    SPI_CHECK(spicommon_periph_claim(host_id, name), "host already in use", ESP_ERR_INVALID_STATE);

    static spicommon_bus_context_t ctx;
    ctx.host_id = host_id;
    bus_ctx[host_id] = &ctx;
    return ESP_OK;
}

esp_err_t SPI_COMMON_ATTR spicommon_bus_free(spi_host_device_t host_id)
{
    assert(bus_ctx[host_id]);
    spicommon_periph_free(host_id);
    bus_ctx[host_id] = NULL;
    return ESP_OK;
}

// DMA support completely removed for polling-only mode

static inline SPI_COMMON_ATTR bool is_valid_host(spi_host_device_t host)
{
#if (SOC_SPI_PERIPH_NUM == 2)
    return host >= SPI1_HOST && host <= SPI2_HOST;
#elif (SOC_SPI_PERIPH_NUM == 3)
    return host >= SPI1_HOST && host <= SPI3_HOST;
#endif
}

static SPI_COMMON_ATTR bool check_iomux_pins_quad(spi_host_device_t host, const spi_bus_config_t* bus_config)
{
    if (bus_config->sclk_io_num >= 0 &&
            bus_config->sclk_io_num != spi_periph_signal[host].spiclk_iomux_pin) {
        return false;
    }
    if (bus_config->quadwp_io_num >= 0 &&
            bus_config->quadwp_io_num != spi_periph_signal[host].spiwp_iomux_pin) {
        return false;
    }
    if (bus_config->quadhd_io_num >= 0 &&
            bus_config->quadhd_io_num != spi_periph_signal[host].spihd_iomux_pin) {
        return false;
    }
    if (bus_config->mosi_io_num >= 0 &&
            bus_config->mosi_io_num != spi_periph_signal[host].spid_iomux_pin) {
        return false;
    }
    if (bus_config->miso_io_num >= 0 &&
            bus_config->miso_io_num != spi_periph_signal[host].spiq_iomux_pin) {
        return false;
    }
    return true;
}

static SPI_COMMON_ATTR bool bus_uses_iomux_pins(spi_host_device_t host, const spi_bus_config_t* bus_config)
{
    return check_iomux_pins_quad(host, bus_config);
}

static void SPI_COMMON_ATTR bus_iomux_pins_set_quad(spi_host_device_t host, const spi_bus_config_t* bus_config)
{
    if (bus_config->mosi_io_num >= 0) {
        gpio_hal_iomux_in(&s_gpio_hal, bus_config->mosi_io_num, spi_periph_signal[host].func, spi_periph_signal[host].spid_in);
        gpio_hal_iomux_out(&s_gpio_hal, bus_config->mosi_io_num, spi_periph_signal[host].func);
    }
    if (bus_config->miso_io_num >= 0) {
        gpio_hal_iomux_in(&s_gpio_hal, bus_config->miso_io_num, spi_periph_signal[host].func, spi_periph_signal[host].spiq_in);
        gpio_hal_iomux_out(&s_gpio_hal, bus_config->miso_io_num, spi_periph_signal[host].func);
    }
    if (bus_config->quadwp_io_num >= 0) {
        gpio_hal_iomux_in(&s_gpio_hal, bus_config->quadwp_io_num, spi_periph_signal[host].func, spi_periph_signal[host].spiwp_in);
        gpio_hal_iomux_out(&s_gpio_hal, bus_config->quadwp_io_num, spi_periph_signal[host].func);
    }
    if (bus_config->quadhd_io_num >= 0) {
        gpio_hal_iomux_in(&s_gpio_hal, bus_config->quadhd_io_num, spi_periph_signal[host].func, spi_periph_signal[host].spihd_in);
        gpio_hal_iomux_out(&s_gpio_hal, bus_config->quadhd_io_num, spi_periph_signal[host].func);
    }
    if (bus_config->sclk_io_num >= 0) {
        gpio_hal_iomux_in(&s_gpio_hal, bus_config->sclk_io_num, spi_periph_signal[host].func, spi_periph_signal[host].spiclk_in);
        gpio_hal_iomux_out(&s_gpio_hal, bus_config->sclk_io_num, spi_periph_signal[host].func);
    }
}

// check if the GPIO is already used by others
static void SPI_COMMON_ATTR s_spi_common_gpio_check_reserve(gpio_num_t gpio_num)
{
    assert(GPIO_IS_VALID_GPIO(gpio_num));  //coverity check
    // Direct implementation: esp_gpio_reserve(BIT64(gpio_num))
    uint64_t orig_occupied_map = s_spi_reserved_pin_mask;
    s_spi_reserved_pin_mask |= BIT64(gpio_num);
    if (orig_occupied_map & BIT64(gpio_num)) {
        ESP_LOGW(SPI_TAG, "GPIO %d is conflict with others and be overwritten", gpio_num);
    }
}

static void SPI_COMMON_ATTR s_spi_common_bus_via_gpio(gpio_num_t gpio_num, int in_sig, int out_sig, uint64_t *io_mask)
{
    assert(GPIO_IS_VALID_GPIO(gpio_num));  //coverity check
    if (in_sig != -1) {
        gpio_hal_input_enable(&s_gpio_hal, gpio_num);
        esp_rom_gpio_connect_in_signal(gpio_num, in_sig, false);
    }
    if (out_sig != -1) {
        // For gpio_matrix, reserve output pins using direct atomic operations
        *io_mask |= BIT64(gpio_num);
        s_spi_common_gpio_check_reserve(gpio_num);
        esp_rom_gpio_connect_out_signal(gpio_num, out_sig, false, false);
    }
    gpio_hal_func_sel(&s_gpio_hal, gpio_num, PIN_FUNC_GPIO);
}

/*
Do the common stuff to hook up a SPI host to a bus defined by a bunch of GPIO pins. Feed it a host number and a
bus config struct and it'll set up the GPIO matrix and enable the device. If a pin is set to non-negative value,
it should be able to be initialized.
*/
esp_err_t SPI_COMMON_ATTR spicommon_bus_initialize_io(spi_host_device_t host, const spi_bus_config_t *bus_config, uint32_t flags, uint32_t* flags_o, uint64_t *io_reserved)
{
    uint32_t temp_flag = 0;
    uint64_t gpio_reserv = 0;
    //check pin capabilities
    if (bus_config->sclk_io_num >= 0) {
        temp_flag |= SPICOMMON_BUSFLAG_SCLK;
        SPI_CHECK_PIN(bus_config->sclk_io_num, "sclk", (flags & SPICOMMON_BUSFLAG_MASTER));
    }
    if (bus_config->quadwp_io_num >= 0) {
        SPI_CHECK_PIN(bus_config->quadwp_io_num, "wp", true);
    }
    if (bus_config->quadhd_io_num >= 0) {
        SPI_CHECK_PIN(bus_config->quadhd_io_num, "hd", true);
    }

    //set flags for QUAD mode according to the existence of wp and hd
    if (bus_config->quadhd_io_num >= 0 && bus_config->quadwp_io_num >= 0) {
        temp_flag |= SPICOMMON_BUSFLAG_WPHD;
    }
    if (bus_config->mosi_io_num >= 0) {
        temp_flag |= SPICOMMON_BUSFLAG_MOSI;
        SPI_CHECK_PIN(bus_config->mosi_io_num, "mosi", (flags & SPICOMMON_BUSFLAG_MASTER) || (temp_flag & SPICOMMON_BUSFLAG_DUAL));
    }
    if (bus_config->miso_io_num >= 0) {
        temp_flag |= SPICOMMON_BUSFLAG_MISO;
        SPI_CHECK_PIN(bus_config->miso_io_num, "miso", !(flags & SPICOMMON_BUSFLAG_MASTER) || (temp_flag & SPICOMMON_BUSFLAG_DUAL));
    }
    //set flags for DUAL mode according to output-capability of MOSI and MISO pins.
    if ((bus_config->mosi_io_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(bus_config->mosi_io_num)) &&
            (bus_config->miso_io_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(bus_config->miso_io_num))) {
        temp_flag |= SPICOMMON_BUSFLAG_DUAL;
    }

    //check if the selected pins correspond to the iomux pins of the peripheral
    bool use_iomux = !(flags & SPICOMMON_BUSFLAG_GPIO_PINS) && bus_uses_iomux_pins(host, bus_config);
    if (use_iomux) {
        temp_flag |= SPICOMMON_BUSFLAG_IOMUX_PINS;
    } else {
        temp_flag |= SPICOMMON_BUSFLAG_GPIO_PINS;
    }

    uint32_t missing_flag = flags & ~temp_flag;
    missing_flag &= ~SPICOMMON_BUSFLAG_MASTER;  //don't check this flag
    missing_flag &= ~SPICOMMON_BUSFLAG_SLP_ALLOW_PD;

    if (missing_flag != 0) {
        //check pins existence
        if (missing_flag & SPICOMMON_BUSFLAG_SCLK) {
            ESP_LOGE(SPI_TAG, "sclk pin required.");
        }
        if (missing_flag & SPICOMMON_BUSFLAG_MOSI) {
            ESP_LOGE(SPI_TAG, "mosi pin required.");
        }
        if (missing_flag & SPICOMMON_BUSFLAG_MISO) {
            ESP_LOGE(SPI_TAG, "miso pin required.");
        }
        if (missing_flag & SPICOMMON_BUSFLAG_DUAL) {
            ESP_LOGE(SPI_TAG, "not both mosi and miso output capable");
        }
        if (missing_flag & SPICOMMON_BUSFLAG_WPHD) {
            ESP_LOGE(SPI_TAG, "both wp and hd required.");
        }
        if (missing_flag & SPICOMMON_BUSFLAG_IOMUX_PINS) {
            ESP_LOGE(SPI_TAG, "not using iomux pins");
        }
        SPI_CHECK(false, "not all required capabilities satisfied.", ESP_ERR_INVALID_ARG);
    }

    ESP_LOGD(SPI_TAG, "SPI%d use %s.", host + 1, use_iomux ? "iomux pins" : "gpio matrix");
    if (use_iomux) {
        //All SPI iomux pin selections resolve to 1, so we put that here instead of trying to figure
        //out which PIN_FUNC_GPIOx_xSPIxx to grab; they all are defined to 1 anyway.
        for (uint8_t i = 0; i < sizeof(bus_config->iocfg) / sizeof(bus_config->iocfg[0]); i++) {
            if (!(flags & SPICOMMON_BUSFLAG_OCTAL) && (i >= 4)) {
                break;
            }
            // For iomux, reserve all configured pins
            if (GPIO_IS_VALID_GPIO(bus_config->iocfg[i])) {
                gpio_reserv |= BIT64(bus_config->iocfg[i]);
                s_spi_common_gpio_check_reserve(bus_config->iocfg[i]);
            }
        }
        bus_iomux_pins_set_quad(host, bus_config);
    } else {
        //Use GPIO matrix
        if (bus_config->mosi_io_num >= 0) {
            int in_sig = (!(flags & SPICOMMON_BUSFLAG_MASTER) || (temp_flag & SPICOMMON_BUSFLAG_DUAL)) ? spi_periph_signal[host].spid_in : -1;
            int out_sig = ((flags & SPICOMMON_BUSFLAG_MASTER) || (temp_flag & SPICOMMON_BUSFLAG_DUAL)) ? spi_periph_signal[host].spid_out : -1;
            s_spi_common_bus_via_gpio(bus_config->mosi_io_num, in_sig, out_sig, &gpio_reserv);
        }
        if (bus_config->miso_io_num >= 0) {
            int in_sig = ((flags & SPICOMMON_BUSFLAG_MASTER) || (temp_flag & SPICOMMON_BUSFLAG_DUAL)) ? spi_periph_signal[host].spiq_in : -1;
            int out_sig = (!(flags & SPICOMMON_BUSFLAG_MASTER) || (temp_flag & SPICOMMON_BUSFLAG_DUAL)) ? spi_periph_signal[host].spiq_out : -1;
            s_spi_common_bus_via_gpio(bus_config->miso_io_num, in_sig, out_sig, &gpio_reserv);
        }
        if (bus_config->sclk_io_num >= 0) {
            int in_sig = (flags & SPICOMMON_BUSFLAG_MASTER) ? -1 : spi_periph_signal[host].spiclk_in;
            int out_sig = (flags & SPICOMMON_BUSFLAG_MASTER) ? spi_periph_signal[host].spiclk_out : -1;
            s_spi_common_bus_via_gpio(bus_config->sclk_io_num, in_sig, out_sig, &gpio_reserv);
        }
        if (bus_config->quadwp_io_num >= 0) {
            s_spi_common_bus_via_gpio(bus_config->quadwp_io_num, spi_periph_signal[host].spiwp_in, spi_periph_signal[host].spiwp_out, &gpio_reserv);
        }
        if (bus_config->quadhd_io_num >= 0) {
            s_spi_common_bus_via_gpio(bus_config->quadhd_io_num, spi_periph_signal[host].spihd_in, spi_periph_signal[host].spihd_out, &gpio_reserv);
        }
    }

    if (flags_o) {
        *flags_o = temp_flag;
    }
    if (io_reserved) {
        *io_reserved |= gpio_reserv;
    }
    return ESP_OK;
}

esp_err_t SPI_COMMON_ATTR spicommon_bus_free_io_cfg(const spi_bus_config_t *bus_cfg, uint64_t *io_reserved)
{
    for (uint8_t i = 0; i < sizeof(bus_cfg->iocfg) / sizeof(bus_cfg->iocfg[0]); i++) {
        if (i > 4) {
            break;
        }
        if (GPIO_IS_VALID_GPIO(bus_cfg->iocfg[i]) && (*io_reserved & BIT64(bus_cfg->iocfg[i]))) {
            *io_reserved &= ~BIT64(bus_cfg->iocfg[i]);
            gpio_hal_output_disable(&s_gpio_hal, bus_cfg->iocfg[i]);
            // Direct implementation: esp_gpio_revoke(BIT64(bus_cfg->iocfg[i]))
            s_spi_reserved_pin_mask &= ~BIT64(bus_cfg->iocfg[i]);
        }
    }
    return ESP_OK;
}

void SPI_COMMON_ATTR spicommon_cs_initialize(spi_host_device_t host, int cs_io_num, int cs_id, int force_gpio_matrix, uint64_t *io_reserved)
{
    uint64_t out_mask = 0;
    if (!force_gpio_matrix && cs_io_num == spi_periph_signal[host].spics0_iomux_pin && cs_id == 0) {
        out_mask |= BIT64(cs_io_num);
        s_spi_common_gpio_check_reserve(cs_io_num);
        //The cs0s for all SPI peripherals map to pin mux source 1, so we use that instead of a define.
        gpio_hal_iomux_in(&s_gpio_hal, cs_io_num, spi_periph_signal[host].func, spi_periph_signal[host].spics_in);
        gpio_hal_iomux_out(&s_gpio_hal, cs_io_num, spi_periph_signal[host].func);
    } else {
        //Use GPIO matrix
        if (GPIO_IS_VALID_OUTPUT_GPIO(cs_io_num)) {
            out_mask |= BIT64(cs_io_num);
            s_spi_common_gpio_check_reserve(cs_io_num);
            esp_rom_gpio_connect_out_signal(cs_io_num, spi_periph_signal[host].spics_out[cs_id], false, false);
        }
        // cs_id 0 is always used by slave for input
        if (cs_id == 0) {
            gpio_hal_input_enable(&s_gpio_hal, cs_io_num);
            esp_rom_gpio_connect_in_signal(cs_io_num, spi_periph_signal[host].spics_in, false);
        }
        gpio_hal_func_sel(&s_gpio_hal, cs_io_num, PIN_FUNC_GPIO);
    }
    if (io_reserved) {
        *io_reserved |= out_mask;
    }
}

void SPI_COMMON_ATTR spicommon_cs_free_io(int cs_gpio_num, uint64_t *io_reserved)
{
    if (GPIO_IS_VALID_GPIO(cs_gpio_num) && (*io_reserved & BIT64(cs_gpio_num))) {
        *io_reserved &= ~BIT64(cs_gpio_num);
        gpio_hal_output_disable(&s_gpio_hal, cs_gpio_num);
        // Direct implementation: esp_gpio_revoke(BIT64(cs_gpio_num))
        s_spi_reserved_pin_mask &= ~BIT64(cs_gpio_num);
    }
}

//----------------------------------------------------------master bus init-------------------------------------------------------//
esp_err_t SPI_COMMON_ATTR spi_bus_initialize(spi_host_device_t host_id, const spi_bus_config_t *bus_config, spi_dma_chan_t dma_chan)
{
    esp_err_t err = ESP_OK;
    SPI_CHECK(is_valid_host(host_id), "invalid host_id", ESP_ERR_INVALID_ARG);
    SPI_CHECK(bus_ctx[host_id] == NULL, "SPI bus already initialized.", ESP_ERR_INVALID_STATE);

    ESP_RETURN_ON_ERROR(spicommon_bus_alloc(host_id, "spi master"), SPI_TAG, "alloc host failed");
    spi_bus_attr_t *bus_attr = (spi_bus_attr_t *)spi_bus_get_attr(host_id);
    spicommon_bus_context_t *ctx = __containerof(bus_attr, spicommon_bus_context_t, bus_attr);
    assert(bus_attr && ctx);  //coverity check
    bus_attr->bus_cfg = *bus_config;

    // Polling-only mode: DMA is always disabled
    if (dma_chan != SPI_DMA_DISABLED) {
        ESP_LOGW(SPI_TAG, "DMA requested but not supported in polling-only mode, using SPI_DMA_DISABLED");
    }
    bus_attr->dma_enabled = 0;
    bus_attr->max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;

    if (bus_attr->bus_cfg.flags & SPICOMMON_BUSFLAG_SLP_ALLOW_PD) {
        ESP_LOGE(SPI_TAG, "power down peripheral in sleep is not enabled or not supported on your target");
    }

    err = spicommon_bus_initialize_io(host_id, bus_config, SPICOMMON_BUSFLAG_MASTER | bus_config->flags, &bus_attr->flags, &bus_attr->gpio_reserve);
    if (err != ESP_OK) {
        goto cleanup;
    }

    return ESP_OK;

cleanup:
    if (bus_attr) {
        // Simplified for polling-only mode - no bus lock or DMA to deinit
    }
    spicommon_bus_free(host_id);
    return err;
}

const SPI_COMMON_ATTR spi_bus_attr_t* spi_bus_get_attr(spi_host_device_t host_id)
{
    if (bus_ctx[host_id] == NULL) {
        return NULL;
    }

    return &bus_ctx[host_id]->bus_attr;
}

esp_err_t SPI_COMMON_ATTR spi_bus_free(spi_host_device_t host_id)
{
    if (bus_ctx[host_id] == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_OK;
    spicommon_bus_context_t* ctx = bus_ctx[host_id];
    spi_bus_attr_t* bus_attr = &ctx->bus_attr;

    // Simplified for polling-only mode - no destroy function
    spicommon_bus_free_io_cfg(&bus_attr->bus_cfg, &bus_attr->gpio_reserve);

    // Simplified for polling-only mode - no bus lock or DMA to deinit
    spicommon_bus_free(host_id);
    return err;
}
