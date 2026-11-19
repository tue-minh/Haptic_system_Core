# TODO: SPI1 Slave Implementation (`STM32_FIRMWARE_SLAVE`)

> **Target MCU**: `STM32F103C8Tx` (`72 MHz` SYSCLK, `SPI1` remapped on `PB3/PB4/PB5`, `SPI1_NSS` on `PB6`)  
> **Goal**: Whenever the Master asserts CS (`PB6` LOW), continuously stream all 5 ADC channels (`PA0`–`PA4`) over `SPI1` using DMA.  
> **Status**: Implemented — EXTI chip-select handling, pre-packed shadow frame and priority tuning are in `Core/Src/main.c`, `Core/Src/gpio.c`, `Core/Src/dma.c`. See [SPI_SLAVE_IMPLEMENTATION_REPORT.md](SPI_SLAVE_IMPLEMENTATION_REPORT.md). Logic-analyzer verification remains.

---

## 1. Current State (Baseline)

- `SPI1` is configured in [spi.c](Core/Src/spi.c) as `SPI_MODE_SLAVE`, `8-bit`, `CPOL=LOW`, `CPHA=1EDGE`, `MSB`, `SPI_NSS_SOFT`, with `DMA1_Channel2` (`RX`) and `DMA1_Channel3` (`TX`) in `DMA_NORMAL` mode.
- `PB6` (`SPI1_NSS`) is configured in [gpio.c](Core/Src/gpio.c) as `GPIO_MODE_INPUT` with `GPIO_PULLUP`.
- `ItvSpiFrame_t` (16 bytes packed) is already defined in [main.c](Core/Src/main.c):
  - `sof0` (`0xAA`), `sof1` (`0x55`)
  - `seq` (`uint8_t`) — increments on each ADC DMA sample batch
  - `status` (`uint8_t`) — bit `0`: `adc_valid`, bit `1`: `adc_error > 0`
  - `adc_raw[5]` (`uint16_t[5]` LE) — 12-bit filtered ADC values (`0..4095`) for `PA0`–`PA4`
  - `crc16` (`uint16_t` LE) — CRC-16/CCITT-FALSE (`poly=0x1021`, `init=0xFFFF`) over bytes `[0..13]`
- Currently, `SPI1_RearmSlaveDMA()` is called in `HAL_SPI_TxRxCpltCallback` and polled in `while (1)` when `PB6 == GPIO_PIN_SET && !BSY`.

---

## 2. TODO List

### Phase A: Deterministic CS (`PB6`) Edge Handling
- [x] **Configure `PB6` (`SPI1_NSS`) as EXTI Rising + Falling** (in `gpio.c` + `stm32f1xx_it.c` `EXTI9_5_IRQHandler`):
  - **Falling edge (`PB6` -> LOW, Master selected slave)**:
    - Ensure `SPI1` DMA is armed and ready if it was idle.
    - Track `spi_cs_active = 1` and reset any stale error flags (`__HAL_SPI_CLEAR_OVRFLAG(&hspi1)`).
  - **Rising edge (`PB6` -> HIGH, Master deselected slave)**:
    - Set `spi_cs_active = 0`.
    - Defer the abort/re-arm to thread context (`spi_cs_restart_pending`), where `HAL_SPI_Abort`/`HAL_SPI_TransmitReceive_DMA` pack a fresh frame (`0xAA 0x55`) so the next CS assertion starts cleanly at byte 0.
- [x] **Protect against bus contention when deselected**:
  - Kept `SPI_NSS_SOFT` because the master CS is wired to `PB6` (not the hardware remapped `PA15`). MISO is only driven while the SPI peripheral is enabled; keep the link point-to-point.

### Phase B: Continuous 5-Channel ADC Streaming While Selected
- [x] **Double-buffered / Ping-Pong TX refresh in `HAL_SPI_TxRxCpltCallback`**:
  - `HAL_ADC_ConvCpltCallback` pre-packs the next `ItvSpiFrame_t` into a shadow buffer (alternating front/back) so the SPI ISR only has to `memcpy` 16 bytes and re-arm.
  - `HAL_SPI_TxRxCpltCallback` keeps the re-arm path minimal (no CRC computation in the ISR).
- [x] **Check DMA & SPI1 IRQ priorities**:
  - `DMA1_Channel2/3_IRQn` and `SPI1_IRQn` = `0`; `ADC1_2_IRQn` and `DMA1_Channel1_IRQn` = `1`; `USART1_IRQn` and `DMA1_Channel4/5_IRQn` = `2`.
- [x] **Handle `HAL_SPI_ErrorCallback`**:
  - On `OVR` / `MODF` / DMA error, clear flags, increment `spi_error_count`, and re-arm if the peripheral is ready; otherwise leave it to the deferred CS restart.

### Phase C: ADC Sampling Rate & Verification
- [x] **Review ADC sample period (`ADC_SAMPLE_PERIOD_MS`)**:
  - Kept at `10 ms` (100 Hz) with the 8-sample moving average. This matches the master poll rate and the 100 Hz ITV feedback requirement; faster sampling only shortens the filter window without adding useful bandwidth.
- [ ] **Test & Verify**:
  - Verify continuous multi-frame burst (e.g. `10 x 16 = 160 bytes` under a single CS LOW pulse) with logic analyzer / master firmware.
  - Confirm `SOF` (`0xAA 0x55`), incrementing `seq`, all 5 channels (`PA0`–`PA4`), and `CRC16` pass 100% of frames without byte shifts.
