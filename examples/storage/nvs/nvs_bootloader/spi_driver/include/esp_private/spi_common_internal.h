/*
 * SPDX-FileCopyrightText: 2010-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Internal header, don't use it in the user code

#pragma once

#include "driver/spi_common.h"
#include "hal/spi_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define GPIO_PIN_COUNT                      (SOC_GPIO_PIN_COUNT)
/// Check whether it is a valid GPIO number
#define GPIO_IS_VALID_GPIO(gpio_num)        ((gpio_num >= 0) && \
                                              (((1ULL << (gpio_num)) & SOC_GPIO_VALID_GPIO_MASK) != 0))
/// Check whether it can be a valid GPIO number of output mode
#define GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) ((gpio_num >= 0) && \
                                              (((1ULL << (gpio_num)) & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) != 0))
/// Check whether it can be a valid digital I/O pad
#define GPIO_IS_VALID_DIGITAL_IO_PAD(gpio_num) ((gpio_num >= 0) && \
                                                 (((1ULL << (gpio_num)) & SOC_GPIO_VALID_DIGITAL_IO_PAD_MASK) != 0))

// Status of a spi bus
typedef enum {
    SPI_BUS_FSM_DISABLED,               ///< Bus is disabled, clock and power is allowed to be closed.
    SPI_BUS_FSM_ENABLED,                ///< Bus is ready to be used
} spi_bus_fsm_t;

/// Attributes of an SPI bus
typedef struct {
    spi_bus_config_t bus_cfg;           ///< Config used to initialize the bus
    uint64_t gpio_reserve;              ///< reserved output gpio bit mask
    uint32_t flags;                     ///< Flags (attributes) of the bus
    int max_transfer_sz;                ///< Maximum length of bytes available to send
    bool dma_enabled;                   ///< To enable DMA or not
    size_t internal_mem_align_size;     ///< Buffer align byte requirement for internal memory
} spi_bus_attr_t;

/**
 * @brief Connect a SPI peripheral to GPIO pins
 *
 * This routine is used to connect a SPI peripheral to the IO-pads and DMA channel given in
 * the arguments. Depending on the IO-pads requested, the routing is done either using the
 * IO_mux or using the GPIO matrix.
 *
 * @param host SPI peripheral to be routed
 * @param bus_config Pointer to a spi_bus_config struct detailing the GPIO pins
 * @param flags Combination of SPICOMMON_BUSFLAG_* flags, set to ensure the pins set are capable with some functions:
 *              - ``SPICOMMON_BUSFLAG_MASTER``: Initialize I/O in master mode
 *              - ``SPICOMMON_BUSFLAG_SLAVE``: Initialize I/O in slave mode
 *              - ``SPICOMMON_BUSFLAG_IOMUX_PINS``: Pins set should match the iomux pins of the controller.
 *              - ``SPICOMMON_BUSFLAG_SCLK``, ``SPICOMMON_BUSFLAG_MISO``, ``SPICOMMON_BUSFLAG_MOSI``:
 *                  Make sure SCLK/MISO/MOSI is/are set to a valid GPIO. Also check output capability according to the mode.
 *              - ``SPICOMMON_BUSFLAG_DUAL``: Make sure both MISO and MOSI are output capable so that DIO mode is capable.
 *              - ``SPICOMMON_BUSFLAG_WPHD`` Make sure WP and HD are set to valid output GPIOs.
 *              - ``SPICOMMON_BUSFLAG_QUAD``: Combination of ``SPICOMMON_BUSFLAG_DUAL`` and ``SPICOMMON_BUSFLAG_WPHD``.
 *              - ``SPICOMMON_BUSFLAG_IO4_IO7``: Make sure spi data4 ~ spi data7 are set to valid output GPIOs.
 *              - ``SPICOMMON_BUSFLAG_OCTAL``: Combination of ``SPICOMMON_BUSFLAG_QUAL`` and ``SPICOMMON_BUSFLAG_IO4_IO7``.
 * @param[out] flags_o A SPICOMMON_BUSFLAG_* flag combination of bus abilities will be written to this address.
 *              Leave to NULL if not needed.
 *              - ``SPICOMMON_BUSFLAG_IOMUX_PINS``: The bus is connected to iomux pins.
 *              - ``SPICOMMON_BUSFLAG_SCLK``, ``SPICOMMON_BUSFLAG_MISO``, ``SPICOMMON_BUSFLAG_MOSI``: The bus has
 *                  CLK/MISO/MOSI connected.
 *              - ``SPICOMMON_BUSFLAG_DUAL``: The bus is capable with DIO mode.
 *              - ``SPICOMMON_BUSFLAG_WPHD`` The bus has WP and HD connected.
 *              - ``SPICOMMON_BUSFLAG_QUAD``: Combination of ``SPICOMMON_BUSFLAG_DUAL`` and ``SPICOMMON_BUSFLAG_WPHD``.
 *              - ``SPICOMMON_BUSFLAG_IO4_IO7``: The bus has spi data4 ~ spi data7 connected.
 *              - ``SPICOMMON_BUSFLAG_OCTAL``: Combination of ``SPICOMMON_BUSFLAG_QUAL`` and ``SPICOMMON_BUSFLAG_IO4_IO7``.
 * @param[out] io_reserved Output the reserved gpio map
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_OK                on success
 */
esp_err_t spicommon_bus_initialize_io(spi_host_device_t host, const spi_bus_config_t *bus_config, uint32_t flags, uint32_t *flags_o,  uint64_t *io_reserved);

/**
 * @brief Free the IO used by a SPI peripheral
 *
 * @param bus_cfg Bus config struct which defines which pins to be used.
 * @param io_reserved Bitmap indicate which pin is reserved
 *
 * @return
 *         - ESP_ERR_INVALID_ARG   if parameter is invalid
 *         - ESP_OK                on success
 */
esp_err_t spicommon_bus_free_io_cfg(const spi_bus_config_t *bus_cfg, uint64_t *io_reserved);

/**
 * @brief Initialize a Chip Select pin for a specific SPI peripheral
 *
 * @param host SPI peripheral
 * @param cs_io_num GPIO pin to route
 * @param cs_id Hardware CS id to route
 * @param force_gpio_matrix If true, CS will always be routed through the GPIO matrix. If false,
 *                          if the GPIO number allows it, the routing will happen through the IO_mux.
 * @param[out] io_reserved Output the reserved gpio map
 */
void spicommon_cs_initialize(spi_host_device_t host, int cs_io_num, int cs_id, int force_gpio_matrix, uint64_t *io_reserved);

/**
 * @brief Free a chip select line
 *
 * @param cs_gpio_num CS gpio num to free
 * @param io_reserved Bitmap indicate which pin is reserved
 */
void spicommon_cs_free_io(int cs_gpio_num, uint64_t *io_reserved);

/*******************************************************************************
 * Bus attributes
 ******************************************************************************/
/**
 * @brief Get the attributes of a specified SPI bus.
 *
 * @param host_id The specified host to get attribute
 * @return (Const) Pointer to the attributes
 */
const spi_bus_attr_t* spi_bus_get_attr(spi_host_device_t host_id);

#ifdef __cplusplus
}
#endif
