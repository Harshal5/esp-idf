/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*******************************************************************************
 * NOTICE
 * The hal is not public api, don't use in application code.
 * See readme.md in soc/include/hal/readme.md
 ******************************************************************************/

// The LL layer for SPI register operations

#pragma once

#include "soc/spi_periph.h"
#include "soc/spi_struct.h"
#include "soc/system_struct.h"
#include "hal/assert.h"
#include "hal/spi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 * Control
 *----------------------------------------------------------------------------*/
/**
 * Enable peripheral register clock
 *
 * @param host_id   Peripheral index number, see `spi_host_device_t`
 * @param enable    Enable/Disable
 */
static inline void _spi_ll_enable_bus_clock(spi_host_device_t host_id, bool enable)
{
    switch (host_id) {
    case SPI1_HOST:
        SYSTEM.perip_clk_en0.reg_spi01_clk_en = enable;
        break;
    case SPI2_HOST:
        SYSTEM.perip_clk_en0.reg_spi2_clk_en = enable;
        break;
    default: HAL_ASSERT(false);
    }
}

/**
 * Reset whole peripheral register to init value defined by HW design
 *
 * @param host_id   Peripheral index number, see `spi_host_device_t`
 */
static inline void _spi_ll_reset_register(spi_host_device_t host_id)
{
    switch (host_id) {
    case SPI1_HOST:
        SYSTEM.perip_rst_en0.reg_spi01_rst = 1;
        SYSTEM.perip_rst_en0.reg_spi01_rst = 0;
        break;
    case SPI2_HOST:
        SYSTEM.perip_rst_en0.reg_spi2_rst = 1;
        SYSTEM.perip_rst_en0.reg_spi2_rst = 0;
        break;
    default: HAL_ASSERT(false);
    }
}

#ifdef __cplusplus
}
#endif
