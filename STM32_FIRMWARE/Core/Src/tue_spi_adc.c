/**
  ******************************************************************************
  * @file    tue_spi_adc.c
  * @brief   SPI master driver for the STM32F103 ITV valve ADC slave.
  ******************************************************************************
  */

#include "tue_spi_adc.h"

#include <string.h>

#include "cmsis_os2.h"
#include "main.h"
#include "spi.h"

#define TUE_SPI_ADC_SPI_TIMEOUT_MS 100U

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint8_t            bus_id;
    uint8_t            cs_idx;
} tue_spi_adc_slave_desc_t;

static const tue_spi_adc_slave_desc_t s_slaves[TUE_SPI_ADC_SLAVE_COUNT] = {
    { &hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, 1U, 0U },
    { &hspi1, SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, 1U, 1U },
    { &hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin, 1U, 2U },
    { &hspi3, SPI3_CS1_GPIO_Port, SPI3_CS1_Pin, 3U, 0U },
    { &hspi3, SPI3_CS2_GPIO_Port, SPI3_CS2_Pin, 3U, 1U },
    { &hspi3, SPI3_CS3_GPIO_Port, SPI3_CS3_Pin, 3U, 2U },
};

static tue_spi_adc_snapshot_t s_cache[TUE_SPI_ADC_SLAVE_COUNT];

static uint8_t s_tx_dummy[TUE_SPI_ADC_BURST_LEN];
static uint8_t s_rx_burst[TUE_SPI_ADC_BURST_LEN];

static bool               s_initialized  = false;
static osThreadId_t       s_task         = NULL;
static volatile uint8_t   s_enabled      = 1U;
static volatile uint8_t   s_bus_id       = 1U;
static volatile uint8_t   s_cs_idx       = 0U;
static volatile uint8_t   s_period_ms    = TUE_SPI_ADC_DEFAULT_PERIOD_MS;
static volatile uint8_t   s_active_idx   = 0U;
static volatile uint32_t  s_total_frames = 0U;

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t crc16_ccitt_false(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static bool parse_burst(const uint8_t *buf, uint32_t len,
                        tue_spi_adc_snapshot_t *out)
{
    bool found = false;
    uint32_t pos = 0U;

    while ((pos + TUE_SPI_ADC_FRAME_LEN) <= len) {
        if ((buf[pos] != TUE_SPI_ADC_SOF0) ||
            (buf[pos + 1U] != TUE_SPI_ADC_SOF1)) {
            pos++;
            continue;
        }

        uint16_t crc_rx = rd_u16(&buf[pos + 14U]);
        uint16_t crc_calc = crc16_ccitt_false(&buf[pos], 14U);
        if (crc_rx != crc_calc) {
            pos++;
            continue;
        }

        out->seq = buf[pos + 2U];
        out->status = buf[pos + 3U];
        for (uint32_t ch = 0U; ch < TUE_SPI_ADC_CHANNEL_COUNT; ch++) {
            out->adc_raw[ch] = rd_u16(&buf[pos + 4U + (ch * 2U)]);
        }
        out->valid = true;
        found = true;
        pos += TUE_SPI_ADC_FRAME_LEN;
    }

    return found;
}

static void cs_setup_delay(void)
{
    for (volatile uint32_t i = 0U; i < 200U; i++) {
        __NOP();
    }
}

static bool poll_slave(uint8_t idx)
{
    const tue_spi_adc_slave_desc_t *desc = &s_slaves[idx];

    memset(s_tx_dummy, 0, sizeof(s_tx_dummy));
    HAL_GPIO_WritePin(desc->cs_port, desc->cs_pin, GPIO_PIN_RESET);
    cs_setup_delay();

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
        desc->spi, s_tx_dummy, s_rx_burst, TUE_SPI_ADC_BURST_LEN,
        TUE_SPI_ADC_SPI_TIMEOUT_MS);

    HAL_GPIO_WritePin(desc->cs_port, desc->cs_pin, GPIO_PIN_SET);

    tue_spi_adc_snapshot_t parsed;
    memset(&parsed, 0, sizeof(parsed));

    if ((st != HAL_OK) || !parse_burst(s_rx_burst, TUE_SPI_ADC_BURST_LEN, &parsed)) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_cache[idx].crc_err_cnt++;
        __set_PRIMASK(primask);
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(s_cache[idx].adc_raw, parsed.adc_raw, sizeof(parsed.adc_raw));
    s_cache[idx].seq            = parsed.seq;
    s_cache[idx].status         = parsed.status;
    s_cache[idx].last_update_ms = HAL_GetTick();
    if (s_cache[idx].rx_ok_cnt < 0xFFFFU) {
        s_cache[idx].rx_ok_cnt++;
    }
    s_cache[idx].valid = true;
    __set_PRIMASK(primask);

    return true;
}

static void tue_spi_adc_task(void *argument)
{
    (void)argument;

    for (;;) {
        uint8_t idx = s_active_idx;
        uint8_t period = s_period_ms;

        if ((s_enabled != 0U) && (idx < TUE_SPI_ADC_SLAVE_COUNT)) {
            (void)poll_slave(idx);
            s_total_frames++;
            osDelay(period);
        } else {
            osDelay(TUE_SPI_ADC_DEFAULT_PERIOD_MS);
        }
    }
}

bool tue_spi_adc_init(void)
{
    if (s_initialized) {
        return true;
    }

    for (uint32_t i = 0U; i < TUE_SPI_ADC_SLAVE_COUNT; i++) {
        HAL_GPIO_WritePin(s_slaves[i].cs_port, s_slaves[i].cs_pin, GPIO_PIN_SET);
    }

    memset(s_cache, 0, sizeof(s_cache));

    s_enabled    = 1U;
    s_bus_id     = 1U;
    s_cs_idx     = 0U;
    s_period_ms  = TUE_SPI_ADC_DEFAULT_PERIOD_MS;
    s_active_idx = 0U;

    const osThreadAttr_t task_attr = {
        .name = "tue_spi_adc",
        .stack_size = 2048,
        .priority = (osPriority_t)osPriorityNormal,
    };
    s_task = osThreadNew(tue_spi_adc_task, NULL, &task_attr);
    s_initialized = (s_task != NULL);
    return s_initialized;
}

uint8_t tue_spi_adc_active_index(void)
{
    return s_active_idx;
}

bool tue_spi_adc_valid_index(uint8_t logical_idx)
{
    return logical_idx < TUE_SPI_ADC_SLAVE_COUNT;
}

void tue_spi_adc_get_snapshot(uint8_t logical_idx, tue_spi_adc_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (logical_idx >= TUE_SPI_ADC_SLAVE_COUNT) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_cache[logical_idx];
    __set_PRIMASK(primask);
}

bool tue_spi_adc_configure(bool enable, uint8_t bus_id, uint8_t cs_idx,
                           uint8_t period_ms)
{
    if ((bus_id != 1U) && (bus_id != 3U)) {
        return false;
    }
    if (cs_idx > 2U) {
        return false;
    }
    if (period_ms == 0U) {
        return false;
    }

    s_enabled    = enable ? 1U : 0U;
    s_bus_id     = bus_id;
    s_cs_idx     = cs_idx;
    s_period_ms  = period_ms;
    s_active_idx = (bus_id == 1U) ? cs_idx : (uint8_t)(3U + cs_idx);
    return true;
}

void tue_spi_adc_get_config(bool *enable, uint8_t *bus_id, uint8_t *cs_idx,
                            uint8_t *period_ms, uint32_t *total_frames)
{
    if (enable != NULL) {
        *enable = (s_enabled != 0U);
    }
    if (bus_id != NULL) {
        *bus_id = s_bus_id;
    }
    if (cs_idx != NULL) {
        *cs_idx = s_cs_idx;
    }
    if (period_ms != NULL) {
        *period_ms = s_period_ms;
    }
    if (total_frames != NULL) {
        *total_frames = s_total_frames;
    }
}
