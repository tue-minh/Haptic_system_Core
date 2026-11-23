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
#define TUE_SPI_ADC_CS_SETUP_US    50U

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} tue_spi_adc_slave_desc_t;

static const tue_spi_adc_slave_desc_t s_slaves[TUE_SPI_ADC_SLAVE_COUNT] = {
    { &hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin },
    { &hspi1, SPI1_CS2_GPIO_Port, SPI1_CS2_Pin },
    { &hspi1, SPI1_CS3_GPIO_Port, SPI1_CS3_Pin },
};

static tue_spi_adc_snapshot_t s_cache[TUE_SPI_ADC_SLAVE_COUNT];

static uint8_t s_tx_dummy[TUE_SPI_ADC_BURST_LEN];
static uint8_t s_rx_burst[TUE_SPI_ADC_BURST_LEN];

static bool               s_initialized  = false;
static osThreadId_t       s_task         = NULL;
static osSemaphoreId_t    s_spi_done_sem = NULL;
static volatile uint8_t   s_spi_xfer_ok  = 0U;
static volatile uint8_t   s_enabled      = 1U;
static volatile uint8_t   s_cs_idx       = 0U;
static volatile uint8_t   s_period_ms    = TUE_SPI_ADC_DEFAULT_PERIOD_MS;
static volatile uint32_t  s_total_frames = 0U;

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/*
 * Parse one received frame.
 *
 * Frame layout (14 bytes, little-endian), no CRC by design:
 *   [0..1] SOF 0xAA 0x55 | [2] seq | [3] status | [4..13] adc_raw[0..4] (u16)
 *
 * The slave sends exactly one frame per chip-select pulse, so the start and
 * length are already guaranteed by the CS framing; only the SOF marker is
 * checked, to catch a wiring/clocking fault rather than silent corruption.
 */
static bool parse_frame(const uint8_t *buf, uint32_t len,
                        tue_spi_adc_snapshot_t *out)
{
    if (len < TUE_SPI_ADC_FRAME_LEN) {
        return false;
    }
    if ((buf[0] != TUE_SPI_ADC_SOF0) || (buf[1] != TUE_SPI_ADC_SOF1)) {
        return false;
    }

    out->seq = buf[2];
    out->status = buf[3];
    for (uint32_t ch = 0U; ch < TUE_SPI_ADC_CHANNEL_COUNT; ch++) {
        out->adc_raw[ch] = rd_u16(&buf[4U + (ch * 2U)]);
    }
    out->valid = true;
    return true;
}

static void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

static void cs_setup_delay(void)
{
    /* Wait a deterministic 50 us after CS falling edge so the F103 slave
     * EXTI ISR can arm its SPI1 DMA with ample margin regardless of -O0/-O2. */
    delay_us(TUE_SPI_ADC_CS_SETUP_US);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        s_spi_xfer_ok = 1U;
        if (s_spi_done_sem != NULL) {
            (void)osSemaphoreRelease(s_spi_done_sem);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        s_spi_xfer_ok = 0U;
        if (s_spi_done_sem != NULL) {
            (void)osSemaphoreRelease(s_spi_done_sem);
        }
    }
}

static bool poll_slave(uint8_t idx)
{
    const tue_spi_adc_slave_desc_t *desc = &s_slaves[idx];

    memset(s_tx_dummy, 0, sizeof(s_tx_dummy));
    memset(s_rx_burst, 0, sizeof(s_rx_burst));

    if (s_spi_done_sem != NULL) {
        while (osSemaphoreAcquire(s_spi_done_sem, 0U) == osOK) {
        }
    }
    s_spi_xfer_ok = 0U;

    HAL_GPIO_WritePin(desc->cs_port, desc->cs_pin, GPIO_PIN_RESET);
    cs_setup_delay();

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(
        desc->spi, s_tx_dummy, s_rx_burst, TUE_SPI_ADC_BURST_LEN);
    if (st == HAL_OK) {
        if ((s_spi_done_sem == NULL) ||
            (osSemaphoreAcquire(s_spi_done_sem, TUE_SPI_ADC_SPI_TIMEOUT_MS) != osOK) ||
            (s_spi_xfer_ok == 0U)) {
            st = HAL_ERROR;
        }
    }

    HAL_GPIO_WritePin(desc->cs_port, desc->cs_pin, GPIO_PIN_SET);
    if (st != HAL_OK) {
        (void)HAL_SPI_Abort(desc->spi);
    }

    tue_spi_adc_snapshot_t parsed;
    memset(&parsed, 0, sizeof(parsed));

    /* Always keep the first bytes of whatever arrived, valid or not: this is
     * what tells a wiring fault apart from a framing error. */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy(s_cache[idx].raw, s_rx_burst,
               TUE_SPI_ADC_RAW_BYTES < TUE_SPI_ADC_BURST_LEN
                   ? TUE_SPI_ADC_RAW_BYTES : TUE_SPI_ADC_BURST_LEN);
        s_cache[idx].raw_len = (uint16_t)TUE_SPI_ADC_BURST_LEN;
        __set_PRIMASK(primask);
    }

    if ((st != HAL_OK) || !parse_frame(s_rx_burst, TUE_SPI_ADC_BURST_LEN, &parsed)) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_cache[idx].crc_err_cnt++;   /* bad-frame counter (no CRC anymore) */
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
    uint32_t next_tick = osKernelGetTickCount();

    for (;;) {
        uint8_t idx = s_cs_idx;
        uint8_t period = s_period_ms;

        if ((s_enabled != 0U) && (idx < TUE_SPI_ADC_SLAVE_COUNT)) {
            (void)poll_slave(idx);
            s_total_frames++;
            next_tick += period;
        } else {
            next_tick += TUE_SPI_ADC_DEFAULT_PERIOD_MS;
        }

        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(next_tick - now) <= 0) {
            next_tick = now + ((s_enabled != 0U) ? period : TUE_SPI_ADC_DEFAULT_PERIOD_MS);
        }
        (void)osDelayUntil(next_tick);
    }
}

bool tue_spi_adc_init(void)
{
    if (s_initialized) {
        return true;
    }

    dwt_enable();

    for (uint32_t i = 0U; i < TUE_SPI_ADC_SLAVE_COUNT; i++) {
        HAL_GPIO_WritePin(s_slaves[i].cs_port, s_slaves[i].cs_pin, GPIO_PIN_SET);
    }

    memset(s_cache, 0, sizeof(s_cache));

    s_enabled    = 1U;
    s_cs_idx     = 0U;
    s_period_ms  = TUE_SPI_ADC_DEFAULT_PERIOD_MS;

    if (s_spi_done_sem == NULL) {
        const osSemaphoreAttr_t sem_attr = { .name = "spi1_adc_done" };
        s_spi_done_sem = osSemaphoreNew(1U, 0U, &sem_attr);
    }

    const osThreadAttr_t task_attr = {
        .name = "tue_spi_adc",
        .stack_size = 2048,
        .priority = (osPriority_t)osPriorityAboveNormal,
    };
    s_task = osThreadNew(tue_spi_adc_task, NULL, &task_attr);
    s_initialized = (s_spi_done_sem != NULL) && (s_task != NULL);
    return s_initialized;
}

uint8_t tue_spi_adc_active_index(void)
{
    return s_cs_idx;
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
    /* SPI1 only. bus_id is kept in the wire protocol for compatibility, but
     * SPI3 belongs to the PS2 receiver and must never be polled as an ADC. */
    if (bus_id != 1U) {
        return false;
    }
    if (cs_idx > 2U) {
        return false;
    }
    if (period_ms == 0U) {
        return false;
    }

    s_enabled    = enable ? 1U : 0U;
    s_cs_idx     = cs_idx;
    s_period_ms  = period_ms;
    return true;
}

void tue_spi_adc_get_config(bool *enable, uint8_t *bus_id, uint8_t *cs_idx,
                            uint8_t *period_ms, uint32_t *total_frames)
{
    if (enable != NULL) {
        *enable = (s_enabled != 0U);
    }
    if (bus_id != NULL) {
        *bus_id = 1U;
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
