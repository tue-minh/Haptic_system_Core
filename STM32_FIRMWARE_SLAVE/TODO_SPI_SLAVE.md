# TODO: SPI1 Slave Implementation (`STM32_FIRMWARE_SLAVE`)

> **Target MCU**: `STM32F103C8Tx` (`72 MHz` SYSCLK, `SPI1` remapped on `PB3/PB4/PB5`, `SPI1_NSS` on `PB6`)  
> **Goal**: Whenever the Master asserts CS (`PB6` LOW), continuously stream all 5 ADC channels (`PA0`–`PA4`) over `SPI1` using DMA.  
> **Status**: Planning / TODO only — do **not** implement code from this document yet.

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
- [ ] **Configure `PB6` (`SPI1_NSS`) as EXTI Rising + Falling** (in `.ioc` or `gpio.c` + `stm32f1xx_it.c` `EXTI9_5_IRQHandler`):
  - **Falling edge (`PB6` -> LOW, Master selected slave)**:
    - Ensure `SPI1` DMA is armed and ready if it was idle.
    - Track `spi_cs_active = 1` and reset any stale error flags (`__HAL_SPI_CLEAR_OVRFLAG(&hspi1)`).
  - **Rising edge (`PB6` -> HIGH, Master deselected slave)**:
    - Set `spi_cs_active = 0`.
    - Abort any partial/in-flight DMA transfer (`HAL_SPI_Abort_IT` or flag for immediate rearm), pack a fresh `ItvSpiFrame_t`, and re-arm `HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, SPI_FRAME_SIZE)` so the next CS assertion starts cleanly at byte 0 (`0xAA 0x55`).
- [ ] **Protect against bus contention when deselected**:
  - Verify `PB4` (`SPI1_MISO`) behavior when `SPI_NSS_SOFT` is used; if multiple slaves share `SPI1` MISO, evaluate switching `SPI1` to `SPI_NSS_HARD_INPUT` (note: hardware `SPI1_NSS` on STM32F103 remapped `SPI1` is `PA15`, whereas `PB6` is software-monitored `SPI1_NSS` — keep `SPI_NSS_SOFT` if wired to `PB6` and only connect point-to-point, or tristate MISO on rising edge if shared).

### Phase B: Continuous 5-Channel ADC Streaming While Selected
- [ ] **Double-buffered / Ping-Pong TX refresh in `HAL_SPI_TxRxCpltCallback`**:
  - While the master holds `PB6` LOW and keeps clocking continuously (`N x 16` bytes), `HAL_SPI_TxRxCpltCallback` fires every 16 bytes.
  - Keep `HAL_SPI_TxRxCpltCallback` as fast as possible (no `HAL_SPI_Abort`, minimal copy) so the next 16-byte DMA transfer is armed within a few microseconds before the master's next byte clock arrives:
    - Prepare the next 16-byte `ItvSpiFrame_t` in a shadow buffer inside the ADC conversion callback (`HAL_ADC_ConvCpltCallback`) or main loop whenever `adc_seq` updates.
    - In `HAL_SPI_TxRxCpltCallback`, copy/swap the pre-packed 16-byte frame into `spi_tx_buf` and immediately call `HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buf, spi_rx_buf, SPI_FRAME_SIZE)`.
- [ ] **Check DMA & SPI1 IRQ priorities**:
  - Ensure `DMA1_Channel2_IRQn`, `DMA1_Channel3_IRQn`, and `SPI1_IRQn` have higher priority (lower numeric value, e.g. `0`) than `ADC1_2_IRQn` / `DMA1_Channel1_IRQn` / `USART1_IRQn` so continuous SPI TX re-arming is never delayed by UART logging.
- [ ] **Handle `HAL_SPI_ErrorCallback`**:
  - On `OVR` / `MODF` / DMA error, clear flags, increment `spi_error_count`, and re-arm only if `PB6 == GPIO_PIN_SET` (or wait for the next rising edge).

### Phase C: ADC Sampling Rate & Verification
- [ ] **Review ADC sample period (`ADC_SAMPLE_PERIOD_MS`)**:
  - Currently `10 ms` (100 Hz) with an 8-sample moving average (`ADC_FILTER_LEN = 8`). Decide if faster updates (e.g. `1–2 ms`) are needed for continuous SPI polling.
- [ ] **Test & Verify**:
  - Verify continuous multi-frame burst (`e.g., 10 x 16 = 160 bytes` under a single CS LOW pulse) with logic analyzer / master firmware.
  - Confirm `SOF` (`0xAA 0x55`), incrementing `seq`, all 5 channels (`PA0`–`PA4`), and `CRC16` pass 100% of frames without byte shifts.
