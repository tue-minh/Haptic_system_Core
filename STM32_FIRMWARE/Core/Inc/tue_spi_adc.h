/**
  ******************************************************************************
  * @file    tue_spi_adc.h
  * @brief   SPI master acquisition of the 16-byte ITV ADC feedback frames.
  ******************************************************************************
  * @attention
  *
  * The STM32F103 slave streams fixed 16-byte frames (ItvSpiFrame_t) that carry
  * the five ITV valve ADC channels. Each poll asserts the slave chip-select,
  * clocks a burst of TUE_SPI_ADC_BURST_FRAMES frames back-to-back, releases CS,
  * and validates every frame with the SOF marker and CRC-16/CCITT-FALSE.
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

#define TUE_SPI_ADC_FRAME_LEN         16U
#define TUE_SPI_ADC_BURST_FRAMES      8U
#define TUE_SPI_ADC_BURST_LEN         (TUE_SPI_ADC_FRAME_LEN * TUE_SPI_ADC_BURST_FRAMES)
#define TUE_SPI_ADC_SLAVE_COUNT       6U
#define TUE_SPI_ADC_CHANNEL_COUNT     5U
#define TUE_SPI_ADC_SOF0              0xAAU
#define TUE_SPI_ADC_SOF1              0x55U
#define TUE_SPI_ADC_STATUS_VALID      0x01U
#define TUE_SPI_ADC_STATUS_ERROR      0x02U
#define TUE_SPI_ADC_DEFAULT_PERIOD_MS 10U

typedef struct {
    uint16_t adc_raw[TUE_SPI_ADC_CHANNEL_COUNT];
    uint8_t  seq;
    uint8_t  status;
    uint16_t rx_ok_cnt;
    uint16_t crc_err_cnt;
    uint32_t last_update_ms;
    bool     valid;
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
