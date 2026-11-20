/**
  ******************************************************************************
  * @file    tue_spi_adc.h
  * @brief   SPI master acquisition of the 16-byte ITV ADC feedback frames.
  ******************************************************************************
  * @attention
  *
  * The STM32F103 slave streams a fixed 14-byte frame (ItvSpiFrame_t) carrying
  * the five ITV valve ADC channels. Each poll asserts the slave chip-select,
  * clocks exactly one frame, releases CS, and sanity-checks the SOF marker.
  * There is no CRC on this link: CS delimits every frame and exactly one frame
  * is transferred per CS pulse, so the framing is already unambiguous.
  * One frame per CS pulse is still required: the slave re-arms its DMA from its
  * completion callback, so it needs the CS-high gap between frames to reload -
  * see TUE_SPI_ADC_BURST_FRAMES.
  *
  * Six logical slaves are mapped onto the two SPI buses:
  *   0..2 -> SPI1 (PB3/PB4/PB5), CS1 PB8, CS2 PB9, CS3 PE0
  *   3..5 -> SPI3 (PC10/PC11/PC12), CS1 PD4, CS2 PD3, CS3 PD2
  *
  ******************************************************************************
  */

#ifndef TUE_SPI_ADC_H
#define TUE_SPI_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define TUE_SPI_ADC_FRAME_LEN         14U
/*
 * Frames clocked per chip-select assertion. MUST stay 1 - see the static assert
 * below for the full reasoning.
 */
#define TUE_SPI_ADC_BURST_FRAMES      1U
#define TUE_SPI_ADC_BURST_LEN         (TUE_SPI_ADC_FRAME_LEN * TUE_SPI_ADC_BURST_FRAMES)
#define TUE_SPI_ADC_SLAVE_COUNT       6U
#define TUE_SPI_ADC_CHANNEL_COUNT     5U
#define TUE_SPI_ADC_SOF0              0xAAU
#define TUE_SPI_ADC_SOF1              0x55U
#define TUE_SPI_ADC_STATUS_VALID      0x01U
#define TUE_SPI_ADC_STATUS_ERROR      0x02U
#define TUE_SPI_ADC_DEFAULT_PERIOD_MS 10U
/* How many received bytes to keep for the 0x504 raw diagnostic frame. */
#define TUE_SPI_ADC_RAW_BYTES         8U

/*
 * WHY ONE FRAME PER CHIP-SELECT (do not raise this to re-enable bursting)
 * ----------------------------------------------------------------------
 * The slave (STM32_FIRMWARE_SLAVE) transfers exactly one 16-byte frame per
 * DMA arm, and re-arms itself from HAL_SPI_TxRxCpltCallback:
 *
 *   spi_tx_buf[16] / spi_rx_buf[16], DMA_NORMAL, re-arm in the completion ISR
 *
 * Two things break when the master clocks several frames inside one CS window:
 *
 *  1. RX buffer overrun. The slave's RX DMA is 16 bytes. Clocking 8 frames
 *     (128 bytes) with CS held low overruns it; bytes 17..128 have nowhere to
 *     go, so the frame-SOF alignment is destroyed on the slave side.
 *
 *  2. HAL busy-waits on BSY inside the DMA ISR. SPI_DMATransmitReceiveCplt()
 *     calls SPI_EndRxTxTransaction(), which spins on SPI_FLAG_BSY (up to
 *     SPI_DEFAULT_TIMEOUT = 100 ms) BEFORE the user completion callback runs.
 *     While the master keeps clocking, BSY never clears, so the slave cannot
 *     re-arm until the whole burst is already over. Everything the master
 *     clocks after the first 16 bytes is whatever the peripheral shifts out
 *     with TXE set - not a valid frame.
 *
 * Net effect at the master: the first frame of each burst is the only one that
 * could parse; the rest are garbage. With no CRC there is nothing to catch it,
 * which is why bursting stays forbidden.
 *
 * One frame per CS gives the slave the deasserted gap it needs to finish its
 * DMA ISR and re-arm before the next frame is clocked. At the default 100 Hz
 * this costs nothing and every poll yields a valid frame.
 */
#if TUE_SPI_ADC_BURST_FRAMES != 1U
#error "TUE_SPI_ADC_BURST_FRAMES must be 1: the slave re-arms one frame per CS"
#endif

typedef struct {
    uint16_t adc_raw[TUE_SPI_ADC_CHANNEL_COUNT];
    uint8_t  seq;
    uint8_t  status;
    uint16_t rx_ok_cnt;
    /* Bad-frame counter. There is no CRC on this link any more, so this counts
     * frames rejected by the SOF sanity check (or a failed SPI transfer). */
    uint16_t crc_err_cnt;
    uint32_t last_update_ms;
    bool     valid;
    /* First TUE_SPI_ADC_RAW_BYTES of the most recent transfer, verbatim.
     * Diagnostic only: it shows what actually arrived, so a wiring/clocking
     * fault (0x00 = MISO low, 0xFF = MISO floating) is distinguishable from a
     * real framing error. Exposed via CAN 0x504. */
    uint8_t  raw[TUE_SPI_ADC_RAW_BYTES];
    uint16_t raw_len;
} tue_spi_adc_snapshot_t;

bool tue_spi_adc_init(void);

uint8_t tue_spi_adc_active_index(void);

bool tue_spi_adc_valid_index(uint8_t logical_idx);

void tue_spi_adc_get_snapshot(uint8_t logical_idx, tue_spi_adc_snapshot_t *out);

bool tue_spi_adc_configure(bool enable, uint8_t bus_id, uint8_t cs_idx,
                           uint8_t period_ms);

void tue_spi_adc_get_config(bool *enable, uint8_t *bus_id, uint8_t *cs_idx,
                            uint8_t *period_ms, uint32_t *total_frames);

#ifdef __cplusplus
}
#endif

#endif /* TUE_SPI_ADC_H */
