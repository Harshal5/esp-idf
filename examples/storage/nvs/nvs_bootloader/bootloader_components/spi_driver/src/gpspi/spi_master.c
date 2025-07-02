/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <sys/param.h>
#include "esp_private/spi_common_internal.h"
#include "hal/clk_tree_hal.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include "hal/spi_hal.h"
#include "hal/spi_ll.h"
#include "hal/hal_utils.h"

#define SPI_MASTER_ATTR IRAM_ATTR

#define SPI_PERIPH_SRC_FREQ_MAX     (80*1000*1000)    //peripheral hardware limitation for clock source into peripheral

static const char *SPI_TAG = "spi_master";
#define SPI_CHECK(a, str, ret_val, ...)  ESP_RETURN_ON_FALSE_ISR(a, ret_val, SPI_TAG, str, ##__VA_ARGS__)

typedef struct spi_device_t spi_device_t;

/// struct to hold private transaction data (like tx and rx buffer for polling-only mode).
typedef struct {
    spi_transaction_t   *trans;
    const uint32_t *buffer_to_send;    //equals to tx_data, if SPI_TRANS_USE_RXDATA is applied; otherwise points to original buffer
    uint32_t *buffer_to_rcv;           //similar to buffer_to_send
    // SCT mode reserved fields removed for polling-only mode
} spi_trans_priv_t;


#define DEV_NUM_MAX 6

typedef struct {
    int id;
    spi_device_t* device[DEV_NUM_MAX];
    intr_handle_t intr;
    spi_hal_context_t hal;
    spi_trans_priv_t cur_trans_buf;
    // SCT mode removed for polling-only mode
    int cur_cs;     //current device doing transaction
    const spi_bus_attr_t* bus_attr;
    // DMA context removed for polling-only mode

    /**
     * Simplified for polling-only mode - just track current device
     * the bus is permanently controlled by a device until `spi_bus_release_bus`` is called. Otherwise
     * the acquiring of SPI bus will be freed when `spi_device_polling_end` is called.
     */
    spi_device_t* current_device;  // Simplified from device_acquiring_lock

//debug information
    bool polling;   //in process of a polling transaction - queues removed for polling-only mode
} spi_host_t;

struct spi_device_t {
    int id;
    // Queues removed for polling-only mode - not needed since transactions are executed synchronously
    spi_device_interface_config_t cfg;
    spi_hal_dev_config_t hal_dev;
    spi_host_t *host;
    // Removed dev_lock for polling-only mode simplification
};

// Static instances for each SPI host
static spi_host_t static_spi_hosts[SOC_SPI_PERIPH_NUM];

static spi_host_t* bus_driver_ctx[SOC_SPI_PERIPH_NUM] = {};

static inline bool is_valid_host(spi_host_device_t host)
{
//SPI1 can be used as GPSPI only on ESP32
#if CONFIG_IDF_TARGET_ESP32
    return host >= SPI1_HOST && host <= SPI3_HOST;
#elif (SOC_SPI_PERIPH_NUM == 2)
    return host == SPI2_HOST;
#elif (SOC_SPI_PERIPH_NUM == 3)
    return host >= SPI2_HOST && host <= SPI3_HOST;
#endif
}

// Should be called before any devices are actually registered or used.
// Currently automatically called after `spi_bus_initialize()` and when first device is registered.
static esp_err_t SPI_MASTER_ATTR spi_master_init_driver(spi_host_device_t host_id)
{
    const spi_bus_attr_t* bus_attr = spi_bus_get_attr(host_id);
    // DMA context removed for polling-only mode
    SPI_CHECK(bus_attr != NULL, "host_id not initialized", ESP_ERR_INVALID_STATE);
    SPI_CHECK(host_id < SOC_SPI_PERIPH_NUM, "invalid host_id", ESP_ERR_INVALID_ARG);

    // Use static instance instead of dynamic allocation
    spi_host_t* host = &static_spi_hosts[host_id];
    // Clear the static structure
    memset(host, 0, sizeof(spi_host_t));

    *host = (spi_host_t) {
        .id = host_id,
        .cur_cs = DEV_NUM_MAX,
        .polling = true,
        .current_device = NULL,
        .bus_attr = bus_attr,
        // DMA context removed for polling-only mode
    };

    // Disable interrupt allocation - using polling mode only
    host->intr = NULL;

    spi_ll_enable_clock(host_id, true);
    spi_hal_init(&host->hal, host_id);
    spi_hal_config_io_default_level(&host->hal, bus_attr->bus_cfg.data_io_default_level);

    bus_driver_ctx[host_id] = host;
    return ESP_OK;
}

/*
 Add a device. This allocates a CS line for the device, allocates memory for the device structure and hooks
 up the CS pin to whatever is specified.
*/
esp_err_t SPI_MASTER_ATTR spi_bus_add_device(spi_host_device_t host_id, const spi_device_interface_config_t *dev_config, spi_device_handle_t *handle)
{
    static spi_device_t dev;
    esp_err_t err = ESP_OK;

    SPI_CHECK(is_valid_host(host_id), "invalid host", ESP_ERR_INVALID_ARG);
    if (bus_driver_ctx[host_id] == NULL) {
        //lazy initialization the driver, get deinitialized by the bus is freed
        err = spi_master_init_driver(host_id);
        if (err != ESP_OK) {
            return err;
        }
    }

    spi_host_t *host = bus_driver_ctx[host_id];
    const spi_bus_attr_t* bus_attr = host->bus_attr;
    SPI_CHECK(dev_config->spics_io_num < 0 || GPIO_IS_VALID_OUTPUT_GPIO(dev_config->spics_io_num), "spics pin invalid", ESP_ERR_INVALID_ARG);
    SPI_CHECK(dev_config->clock_speed_hz > 0, "invalid sclk speed", ESP_ERR_INVALID_ARG);

    uint32_t clock_source_hz = 0;
    uint32_t clock_source_div = 1;
    spi_clock_source_t clk_src = dev_config->clock_source ? dev_config->clock_source : SPI_CLK_SRC_DEFAULT;
    clock_source_hz = clk_hal_apb_get_freq_hz(); // TODO: check if this works for the bootloader

    SPI_CHECK(dev_config->clock_speed_hz <= clock_source_hz, "invalid sclk speed", ESP_ERR_INVALID_ARG);

    //Check post_cb status when `SPI_DEVICE_NO_RETURN_RESULT` flag is set.
    if (dev_config->flags & SPI_DEVICE_NO_RETURN_RESULT) {
        SPI_CHECK(dev_config->post_cb != NULL, "use feature flag 'SPI_DEVICE_NO_RETURN_RESULT' but no post callback function sets", ESP_ERR_INVALID_ARG);
    }

    // Simplified device registration for polling-only mode - removed bus lock registration
    // Find a free device slot
    int freecs = -1;
    for (int i = 0; i < DEV_NUM_MAX; i++) {
        if (host->device[i] == NULL) {
            freecs = i;
            break;
        }
    }
    SPI_CHECK(freecs != -1, "no free cs pins for the host", ESP_ERR_NOT_FOUND);

    //input parameters to calculate timing configuration
    int half_duplex = dev_config->flags & SPI_DEVICE_HALFDUPLEX ? 1 : 0;
    int no_compensate = dev_config->flags & SPI_DEVICE_NO_DUMMY ? 1 : 0;
    int duty_cycle = (dev_config->duty_cycle_pos == 0) ? 128 : dev_config->duty_cycle_pos;
    int use_gpio = !(bus_attr->flags & SPICOMMON_BUSFLAG_IOMUX_PINS);
    spi_hal_timing_param_t timing_param = {
        .half_duplex = half_duplex,
        .no_compensate = no_compensate,
        .clk_src_hz = clock_source_hz,
        .expected_freq = dev_config->clock_speed_hz,
        .duty_cycle = duty_cycle,
        .input_delay_ns = dev_config->input_delay_ns,
        .use_gpio = use_gpio
    };

    //output values of timing configuration
    spi_hal_timing_conf_t temp_timing_conf;
    esp_err_t ret = spi_hal_cal_clock_conf(&timing_param, &temp_timing_conf);
    SPI_CHECK(ret == ESP_OK, "assigned clock speed not supported", ret);
    temp_timing_conf.clock_source = clk_src;
    temp_timing_conf.source_pre_div = clock_source_div;
    temp_timing_conf.source_real_freq = clock_source_hz;
    temp_timing_conf.rx_sample_point = dev_config->sample_point;
    if (temp_timing_conf.rx_sample_point == SPI_SAMPLING_POINT_PHASE_1) {
        SPI_CHECK(spi_ll_master_is_rx_std_sample_supported(), "SPI_SAMPLING_POINT_PHASE_1 is not supported on this chip", ESP_ERR_NOT_SUPPORTED);
    }

    //Allocate memory for device
    dev.id = freecs;

    // Polling-only mode: queues removed - transactions are executed synchronously

    //We want to save a copy of the dev config in the dev struct.
    memcpy(&dev.cfg, dev_config, sizeof(spi_device_interface_config_t));
    dev.cfg.duty_cycle_pos = duty_cycle;
    // TODO: if we have to change the apb clock among transactions, re-calculate this each time the apb clock lock is locked.

    //Set CS pin, CS options
    if (dev_config->spics_io_num >= 0) {
        spicommon_cs_initialize(host_id, dev_config->spics_io_num, freecs, use_gpio, (uint64_t *)&bus_attr->gpio_reserve);
    }

    //save a pointer to device in spi_host_t
    host->device[freecs] = &dev;
    //save a pointer to host in spi_device_t
    dev.host = host;

    //initialise the device specific configuration
    spi_hal_dev_config_t *hal_dev = &(dev.hal_dev);
    hal_dev->mode = dev_config->mode;
    hal_dev->cs_setup = dev_config->cs_ena_pretrans;
    hal_dev->cs_hold = dev_config->cs_ena_posttrans;
    //set hold_time to 0 will not actually append delay to CS
    //set it to 1 since we do need at least one clock of hold time in most cases
    if (hal_dev->cs_hold == 0) {
        hal_dev->cs_hold = 1;
    }
    hal_dev->cs_pin_id = dev.id;
    hal_dev->timing_conf = temp_timing_conf;
    hal_dev->sio = (dev_config->flags) & SPI_DEVICE_3WIRE ? 1 : 0;
    hal_dev->half_duplex = dev_config->flags & SPI_DEVICE_HALFDUPLEX ? 1 : 0;
    hal_dev->tx_lsbfirst = dev_config->flags & SPI_DEVICE_TXBIT_LSBFIRST ? 1 : 0;
    hal_dev->rx_lsbfirst = dev_config->flags & SPI_DEVICE_RXBIT_LSBFIRST ? 1 : 0;
    hal_dev->no_compensate = dev_config->flags & SPI_DEVICE_NO_DUMMY ? 1 : 0;
#if SOC_SPI_AS_CS_SUPPORTED
    hal_dev->as_cs = dev_config->flags & SPI_DEVICE_CLK_AS_CS ? 1 : 0;
#endif
    hal_dev->positive_cs = dev_config->flags & SPI_DEVICE_POSITIVE_CS ? 1 : 0;

    *handle = &dev;
    ESP_LOGD(SPI_TAG, "SPI%d: New device added to CS%d, effective clock: %d Hz", host_id + 1, freecs, temp_timing_conf.real_freq);

    return ESP_OK;
}

esp_err_t SPI_MASTER_ATTR spi_bus_remove_device(spi_device_handle_t handle)
{
    SPI_CHECK(handle != NULL, "invalid handle", ESP_ERR_INVALID_ARG);
    //These checks aren't exhaustive; another thread could sneak in a transaction in between. These are only here to
    //catch design errors and aren't meant to be triggered during normal operation.
    //NOTE: In polling-only mode, queue checks are removed since queues are not used
    SPI_CHECK(handle->host->cur_cs == DEV_NUM_MAX || handle->host->device[handle->host->cur_cs] != handle, "Have unfinished transactions", ESP_ERR_INVALID_STATE);

    //return
    int spics_io_num = handle->cfg.spics_io_num;
    const spi_bus_attr_t* bus_attr = handle->host->bus_attr;
    if (spics_io_num >= 0) {
        spicommon_cs_free_io(spics_io_num, (uint64_t *)&bus_attr->gpio_reserve);
    }

    // Polling-only mode: no queues to delete
    // Removed bus lock unregistration for polling-only mode

    assert(handle->host->device[handle->id] == handle);
    handle->host->device[handle->id] = NULL;
    // Clear the static device structure instead of freeing
    memset(handle, 0, sizeof(spi_device_t));
    return ESP_OK;
}

esp_err_t SPI_MASTER_ATTR spi_device_get_actual_freq(spi_device_handle_t handle, int* freq_khz)
{
    if ((spi_device_t *)handle == NULL || freq_khz == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *freq_khz = handle->hal_dev.timing_conf.real_freq / 1000;
    return ESP_OK;
}

int SPI_MASTER_ATTR spi_get_actual_clock(int fapb, int hz, int duty_cycle)
{
    return spi_hal_master_cal_clock(fapb, hz, duty_cycle);
}

// Setup the device-specified configuration registers. Called every time a new
// transaction is to be sent, but only apply new configurations when the device
// changes or timing change is required.
static SPI_MASTER_ATTR void spi_setup_device(spi_device_t *dev, spi_trans_priv_t *trans_buf)
{
    spi_hal_context_t *hal = &dev->host->hal;
    spi_hal_dev_config_t *hal_dev = &(dev->hal_dev);

    bool clock_changed = false;
    // check if timing config update is required
    if (trans_buf && (trans_buf->trans->override_freq_hz > 0) && (hal_dev->timing_conf.expect_freq != trans_buf->trans->override_freq_hz)) {
        spi_hal_timing_param_t timing_param = {
            .expected_freq = trans_buf->trans->override_freq_hz,
            .clk_src_hz = dev->hal_dev.timing_conf.source_real_freq,
            .duty_cycle = dev->cfg.duty_cycle_pos,
            .input_delay_ns = dev->cfg.input_delay_ns,
            .half_duplex = dev->hal_dev.half_duplex,
            .use_gpio = !(dev->host->bus_attr->flags & SPICOMMON_BUSFLAG_IOMUX_PINS),
        };

        if (ESP_OK == spi_hal_cal_clock_conf(&timing_param, &dev->hal_dev.timing_conf)) {
            clock_changed = true;
        } else {
            ESP_EARLY_LOGW(SPI_TAG, "assigned clock speed %d not supported", trans_buf->trans->override_freq_hz);
        }
    }

    // In polling-only mode, always apply configuration for simplicity
    if (true || clock_changed) {
        /* Configuration has not been applied yet. */
        spi_hal_setup_device(hal, hal_dev);
        spi_ll_set_clk_source(hal->hw, hal_dev->timing_conf.clock_source);
    }
}

static SPI_MASTER_ATTR spi_device_t *get_acquiring_dev(spi_host_t *host)
{
    // Simplified for polling-only mode - just return current device
    return host->current_device;
}

// Debug only
// NOTE if the acquiring is not fully completed, `spi_bus_lock_get_acquiring_dev`
// may return a false `NULL` cause the function returning false `false`.
static inline SPI_MASTER_ATTR bool spi_bus_device_is_polling(spi_device_t *dev)
{
    return get_acquiring_dev(dev->host) == dev && dev->host->polling;
}

/*-----------------------------------------------------------------------------
    Working Functions
-----------------------------------------------------------------------------*/

static void SPI_MASTER_ATTR s_spi_prepare_data(spi_device_t *dev, const spi_hal_trans_config_t *hal_trans)
{
    spi_host_t *host = dev->host;
    spi_hal_dev_config_t *hal_dev = &(dev->hal_dev);
    spi_hal_context_t *hal = &(host->hal);

    // Polling-only mode: always use FIFO, never DMA
    spi_hal_push_tx_buffer(hal, hal_trans);

    spi_hal_enable_data_line(hal->hw, (!hal_dev->half_duplex && hal_trans->rcv_buffer) || hal_trans->send_buffer, !!hal_trans->rcv_buffer);
}

static void SPI_MASTER_ATTR spi_format_hal_trans_struct(spi_device_t *dev, spi_trans_priv_t *trans_buf, spi_hal_trans_config_t *hal_trans)
{
    spi_host_t *host = dev->host;
    spi_transaction_t *trans = trans_buf->trans;
    hal_trans->tx_bitlen = trans->length;
    hal_trans->rx_bitlen = trans->rxlength;
    hal_trans->rcv_buffer = (uint8_t*)host->cur_trans_buf.buffer_to_rcv;
    hal_trans->send_buffer = (uint8_t*)host->cur_trans_buf.buffer_to_send;
    hal_trans->cmd = trans->cmd;
    hal_trans->addr = trans->addr;

    if (trans->flags & SPI_TRANS_VARIABLE_CMD) {
        hal_trans->cmd_bits = ((spi_transaction_ext_t *)trans)->command_bits;
    } else {
        hal_trans->cmd_bits = dev->cfg.command_bits;
    }
    if (trans->flags & SPI_TRANS_VARIABLE_ADDR) {
        hal_trans->addr_bits = ((spi_transaction_ext_t *)trans)->address_bits;
    } else {
        hal_trans->addr_bits = dev->cfg.address_bits;
    }
    if (trans->flags & SPI_TRANS_VARIABLE_DUMMY) {
        hal_trans->dummy_bits = ((spi_transaction_ext_t *)trans)->dummy_bits;
    } else {
        hal_trans->dummy_bits = dev->cfg.dummy_bits;
    }

    hal_trans->cs_keep_active = (trans->flags & SPI_TRANS_CS_KEEP_ACTIVE) ? 1 : 0;
    //Set up OIO/QIO/DIO if needed
    hal_trans->line_mode.data_lines = (trans->flags & SPI_TRANS_MODE_DIO) ? 2 : (trans->flags & SPI_TRANS_MODE_QIO) ? 4 : 1;
    hal_trans->line_mode.addr_lines = (trans->flags & SPI_TRANS_MULTILINE_ADDR) ? hal_trans->line_mode.data_lines : 1;
    hal_trans->line_mode.cmd_lines = (trans->flags & SPI_TRANS_MULTILINE_CMD) ? hal_trans->line_mode.data_lines : 1;
}

// The function is called to send a new transaction, in ISR or in the task.
// Setup the transaction-specified registers and linked-list used by the DMA (or FIFO if DMA is not used)
static void SPI_MASTER_ATTR spi_new_trans(spi_device_t *dev, spi_trans_priv_t *trans_buf)
{
    spi_transaction_t *trans = trans_buf->trans;
    spi_hal_context_t *hal = &(dev->host->hal);
    spi_hal_dev_config_t *hal_dev = &(dev->hal_dev);

    dev->host->cur_cs = dev->id;

    //Reconfigure according to device settings, the function only has effect when the dev_id is changed.
    spi_setup_device(dev, trans_buf);

    //set the transaction specific configuration each time before a transaction setup
    spi_hal_trans_config_t hal_trans = {};
    spi_format_hal_trans_struct(dev, trans_buf, &hal_trans);
    spi_hal_setup_trans(hal, hal_dev, &hal_trans);
    s_spi_prepare_data(dev, &hal_trans);

    //Call pre-transmission callback, if any
    if (dev->cfg.pre_cb) {
        dev->cfg.pre_cb(trans);
    }
    //Kick off transfer
    spi_hal_user_start(hal);
}

// The function is called when a transaction is done, in ISR or in the task.
// Fetch the data from FIFO and call the ``post_cb``.
static void SPI_MASTER_ATTR spi_post_trans(spi_host_t *host)
{
    spi_transaction_t *cur_trans = host->cur_trans_buf.trans;

    // Polling-only mode: always fetch data from FIFO
    spi_hal_fetch_result(&host->hal);
    
    //Call post-transaction callback, if any
    spi_device_t* dev = host->device[host->cur_cs];
    if (dev->cfg.post_cb) {
        dev->cfg.post_cb(cur_trans);
    }

    host->cur_cs = DEV_NUM_MAX;
}

static SPI_MASTER_ATTR esp_err_t check_trans_valid(spi_device_handle_t handle, spi_transaction_t *trans_desc)
{
    SPI_CHECK(handle != NULL, "invalid dev handle", ESP_ERR_INVALID_ARG);
    spi_host_t *host = handle->host;
    const spi_bus_attr_t* bus_attr = host->bus_attr;
    bool tx_enabled = (trans_desc->flags & SPI_TRANS_USE_TXDATA) || (trans_desc->tx_buffer);
    bool rx_enabled = (trans_desc->flags & SPI_TRANS_USE_RXDATA) || (trans_desc->rx_buffer);
    spi_transaction_ext_t *t_ext = (spi_transaction_ext_t *)trans_desc;
    bool dummy_enabled = (((trans_desc->flags & SPI_TRANS_VARIABLE_DUMMY) ? t_ext->dummy_bits : handle->cfg.dummy_bits) != 0);
    bool extra_dummy_enabled = handle->hal_dev.timing_conf.timing_dummy;
    bool is_half_duplex = ((handle->cfg.flags & SPI_DEVICE_HALFDUPLEX) != 0);

    //check transmission length
    SPI_CHECK((trans_desc->flags & SPI_TRANS_USE_RXDATA) == 0 || trans_desc->rxlength <= 32, "SPI_TRANS_USE_RXDATA only available for rxdata transfer <= 32 bits", ESP_ERR_INVALID_ARG);
    SPI_CHECK((trans_desc->flags & SPI_TRANS_USE_TXDATA) == 0 || trans_desc->length <= 32, "SPI_TRANS_USE_TXDATA only available for txdata transfer <= 32 bits", ESP_ERR_INVALID_ARG);
    SPI_CHECK(trans_desc->length <= bus_attr->max_transfer_sz * 8, "txdata transfer > host maximum", ESP_ERR_INVALID_ARG);
    SPI_CHECK(trans_desc->rxlength <= bus_attr->max_transfer_sz * 8, "rxdata transfer > host maximum", ESP_ERR_INVALID_ARG);
    SPI_CHECK(is_half_duplex || trans_desc->rxlength <= trans_desc->length, "rx length > tx length in full duplex mode", ESP_ERR_INVALID_ARG);
    SPI_CHECK(!((trans_desc->flags & (SPI_TRANS_MODE_DIO | SPI_TRANS_MODE_QIO)) && (handle->cfg.flags & SPI_DEVICE_3WIRE)), "Incompatible when setting to both multi-line mode and 3-wire-mode", ESP_ERR_INVALID_ARG);
    SPI_CHECK(!((trans_desc->flags & (SPI_TRANS_MODE_DIO | SPI_TRANS_MODE_QIO)) && !is_half_duplex), "Incompatible when setting to both multi-line mode and half duplex mode", ESP_ERR_INVALID_ARG);

    //MOSI phase is skipped only when both tx_buffer and SPI_TRANS_USE_TXDATA are not set.
    SPI_CHECK(trans_desc->length != 0 || !tx_enabled, "trans tx_buffer should be NULL and SPI_TRANS_USE_TXDATA should be cleared to skip MOSI phase.", ESP_ERR_INVALID_ARG);
    //MISO phase is skipped only when both rx_buffer and SPI_TRANS_USE_RXDATA are not set.
    //If set rxlength=0 in full_duplex mode, it will be automatically set to length
    SPI_CHECK(!is_half_duplex || trans_desc->rxlength != 0 || !rx_enabled, "trans rx_buffer should be NULL and SPI_TRANS_USE_RXDATA should be cleared to skip MISO phase.", ESP_ERR_INVALID_ARG);
    //In Full duplex mode, default rxlength to be the same as length, if not filled in.
    // set rxlength to length is ok, even when rx buffer=NULL
    if (trans_desc->rxlength == 0 && !is_half_duplex) {
        trans_desc->rxlength = trans_desc->length;
    }
    //Dummy phase is not available when both data out and in are enabled, regardless of FD or HD mode.
    SPI_CHECK(!tx_enabled || !rx_enabled || !dummy_enabled || !extra_dummy_enabled, "Dummy phase not available when both data out and in are enabled", ESP_ERR_INVALID_ARG);

    // Polling-only mode: always use FIFO limits
    SPI_CHECK(trans_desc->length <= SPI_LL_CPU_MAX_BIT_LEN, "txdata transfer > hardware max supported len", ESP_ERR_INVALID_ARG);
    SPI_CHECK(trans_desc->rxlength <= SPI_LL_CPU_MAX_BIT_LEN, "rxdata transfer > hardware max supported len", ESP_ERR_INVALID_ARG);

    return ESP_OK;
}

static SPI_MASTER_ATTR esp_err_t setup_priv_desc(spi_host_t *host, spi_trans_priv_t* priv_desc)
{
    spi_transaction_t *trans_desc = priv_desc->trans;

    // rx memory assign - polling-only mode uses original buffers directly
    uint32_t* rcv_ptr;
    if (trans_desc->flags & SPI_TRANS_USE_RXDATA) {
        rcv_ptr = (uint32_t *)&trans_desc->rx_data[0];
    } else {
        //if not use RXDATA neither rx_buffer, buffer_to_rcv assigned to NULL
        rcv_ptr = trans_desc->rx_buffer;
    }

    // tx memory assign - polling-only mode uses original buffers directly
    const uint32_t *send_ptr;
    if (trans_desc->flags & SPI_TRANS_USE_TXDATA) {
        send_ptr = (uint32_t *)&trans_desc->tx_data[0];
    } else {
        //if not use TXDATA neither tx_buffer, tx data assigned to NULL
        send_ptr = trans_desc->tx_buffer ;
    }

    // Polling-only mode: use buffers directly, no DMA alignment or capability checks needed
    priv_desc->buffer_to_send = send_ptr;
    priv_desc->buffer_to_rcv = rcv_ptr;
    return ESP_OK;
}

esp_err_t SPI_MASTER_ATTR spi_device_queue_trans(spi_device_handle_t handle, spi_transaction_t *trans_desc)
{
    // TODO: bootloader DRAM overflow (reduce logs)
    // esp_err_t ret = check_trans_valid(handle, trans_desc);
    // if (ret != ESP_OK) {
    //     return ret;
    // }

    spi_host_t *host = handle->host;

    SPI_CHECK(!spi_bus_device_is_polling(handle), "Cannot queue new transaction while previous polling transaction is not terminated.", ESP_ERR_INVALID_STATE);

    /* CS can only be kept activated if the bus has been acquired with `spi_device_acquire_bus()` first. */
    if (host->current_device != handle && (trans_desc->flags & SPI_TRANS_CS_KEEP_ACTIVE)) {
        return ESP_ERR_INVALID_ARG;
    }

    // In polling-only mode, execute transaction immediately instead of using interrupt-based queuing
    return spi_device_polling_transmit(handle, trans_desc);
}

//Porcelain to do one blocking transmission.
esp_err_t SPI_MASTER_ATTR spi_device_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc)
{
    // In polling-only mode, spi_device_queue_trans executes the transaction immediately
    return spi_device_queue_trans(handle, trans_desc);
}

esp_err_t SPI_MASTER_ATTR spi_device_polling_start(spi_device_handle_t handle, spi_transaction_t *trans_desc)
{
    // TODO: bootloader DRAM overflow (reduce logs)
    esp_err_t ret;
    // ret = check_trans_valid(handle, trans_desc);
    // if (ret != ESP_OK) {
    //     return ret;
    // }
    SPI_CHECK(!spi_bus_device_is_polling(handle), "Cannot send polling transaction while the previous polling transaction is not terminated.", ESP_ERR_INVALID_STATE);

    spi_host_t *host = handle->host;
    spi_trans_priv_t priv_polling_trans = { .trans = trans_desc, };
    ret = setup_priv_desc(host, &priv_polling_trans);
    if (ret != ESP_OK) {
        return ret;
    }

    /* If current_device is set to handle, it means that the user has already
     * acquired the bus thanks to the function `spi_device_acquire_bus()`.
     * In that case, we don't need to take the lock again. */
    if (host->current_device != handle) {
        /* The user cannot ask for the CS to keep active if the bus is not locked/acquired. */
        if ((trans_desc->flags & SPI_TRANS_CS_KEEP_ACTIVE) != 0) {
            ret = ESP_ERR_INVALID_ARG;
        } else {
            // Simplified acquisition for polling-only mode
            if (host->current_device != NULL) {
                ret = ESP_ERR_INVALID_STATE; // Bus already acquired by another device  
            } else {
                host->current_device = handle;
                ret = ESP_OK;
            }
        }
    } else {
        // Device already has the bus, no need to wait
        ret = ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGE(SPI_TAG, "polling can't get buslock");
        return ret;
    }
    //After holding the buslock, common resource can be accessed !!

    //Polling, no interrupt is used.
    host->polling = true;
    host->cur_trans_buf = priv_polling_trans;

    ESP_LOGV(SPI_TAG, "polling trans");
    spi_new_trans(handle, &host->cur_trans_buf);

    return ESP_OK;
}

esp_err_t SPI_MASTER_ATTR spi_device_polling_end(spi_device_handle_t handle)
{
    SPI_CHECK(handle != NULL, "invalid dev handle", ESP_ERR_INVALID_ARG);
    spi_host_t *host = handle->host;

    assert(host->cur_cs == handle->id);
    assert(handle == get_acquiring_dev(host));

    while (!spi_hal_usr_is_done(&host->hal)) {
        esp_rom_delay_us(100);
    }

    // Cache sync not needed in polling-only mode since DMA is disabled

    ESP_LOGV(SPI_TAG, "polling trans done");
    //deal with the in-flight transaction
    spi_post_trans(host);

    host->polling = false;
    return ESP_OK;
}

esp_err_t SPI_MASTER_ATTR spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t* trans_desc)
{
    esp_err_t ret;
    ret = spi_device_polling_start(handle, trans_desc);
    if (ret != ESP_OK) {
        return ret;
    }

    return spi_device_polling_end(handle);
}
