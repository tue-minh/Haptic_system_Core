# SPI1 Slave Implementation Report (`STM32_FIRMWARE_SLAVE`)

> Completes [TODO_SPI_SLAVE.md](TODO_SPI_SLAVE.md). The STM32F103 now reacts
> deterministically to the master chip-select on `PB6` (EXTI), keeps a pre-packed
> `ItvSpiFrame_t` ready in a shadow buffer, and streams 16-byte ADC frames from `SPI1`
> DMA/IRQ with priorities tuned so re-arming is never blocked by logging.

---

## 1. What changed

| File | Change |
| :--- | :--- |
| `Core/Src/gpio.c` | `PB6` (`SPI1_NSS`) changed from `GPIO_MODE_INPUT` to `GPIO_MODE_IT_RISING_FALLING` with pull-up; `EXTI9_5_IRQn` enabled at priority 0 |
| `Core/Src/stm32f1xx_it.c` | New `EXTI9_5_IRQHandler()` dispatching `SPI1_NSS_Pin` to `HAL_GPIO_EXTI_IRQHandler()` |
| `Core/Inc/stm32f1xx_it.h` | `EXTI9_5_IRQHandler` prototype |
| `Core/Src/main.c` | Shadow frame buffers, CS edge callback, deferred restart, ISR fast path |
| `Core/Src/dma.c` | DMA priorities: SPI RX/TX = 0, ADC = 1, USART1 = 2 |
| `Core/Src/adc.c` | `ADC1_2_IRQn` priority = 1 |
| `Core/Src/usart.c` | `USART1_IRQn` priority = 2 |
| Docs | `TODO_SPI_SLAVE.md` status/checkboxes, `FIRMWARE_SYSTEM.md` §4 |

## 2. Chip-select (CS) handling

`HAL_GPIO_EXTI_Callback()` handles `PB6`:

- **Falling edge (master selects the slave)**
  - `spi_cs_active = 1`
  - `__HAL_SPI_CLEAR_OVRFLAG(&hspi1)` to drop any stale overrun from the idle gap.
  - If the SPI peripheral is `READY`, `SPI1_RearmSlaveDMA()` immediately (arm from byte 0).
- **Rising edge (master releases the slave)**
  - `spi_cs_active = 0`
  - `spi_cs_restart_pending = 1` — abort + re-arm happen in the main loop
    (`HAL_SPI_Abort()` is a thread-context call), so the next assertion begins at a fresh
    `0xAA 0x55` frame.
- `SPI1_RearmSlaveDMA()` refuses to abort an in-flight transfer while
  `spi_cs_active != 0`; when CS is already asserted the existing arm is kept. This closes
  the race where a deferred rising-edge restart would abort a newly started burst.

## 3. Pre-packed shadow frame

- `spi_frame_shadow[2][16]` + `spi_frame_front`.
- `HAL_ADC_ConvCpltCallback()` increments `adc_seq`, filters `PA0`–`PA4`, then
  `SPI1_PackFrame()` into the **back** slot and publishes it by flipping `spi_frame_front`.
  CRC-16 work stays out of the SPI ISR.
- `HAL_SPI_TxRxCpltCallback()` copies the published 16 bytes into `spi_tx_buf` and calls
  `HAL_SPI_TransmitReceive_DMA()` again — no CRC loop, no HAL abort in this path.
- `HAL_SPI_ErrorCallback()` clears `OVR`, counts the error, re-arms when ready, otherwise
  sets `spi_cs_restart_pending` for the main loop.

## 4. Interrupt priority map

| IRQ | Priority | Rationale |
| :--- | :--- | :--- |
| `DMA1_Channel2_IRQn` (SPI1 RX) | 0 | Must finish the DMA and re-arm first |
| `DMA1_Channel3_IRQn` (SPI1 TX) | 0 | Same |
| `SPI1_IRQn` | 0 | Calls `HAL_SPI_TxRxCpltCallback` |
| `ADC1_2_IRQn` | 1 | Frame packing; can be preempted by SPI |
| `DMA1_Channel1_IRQn` (ADC) | 1 | Same |
| `USART1_IRQn` | 2 | Debug CLI must never delay SPI re-arming |
| `DMA1_Channel4/5_IRQn` (USART1) | 2 | Same |

## 5. ADC rate

`ADC_SAMPLE_PERIOD_MS` remains `10 ms` (100 Hz) with the 8-sample moving average. This
matches the master poll rate; faster sampling would only shorten the averaging window
without adding useful feedback bandwidth.

## 6. Build verification

`arm-none-eabi-gcc 15.2.1` + CMake/Ninja, Debug:

```text
FLASH:  25,868 B / 64 KB (39.47%)
RAM:     3,104 B / 20 KB (15.16%)
```

> Remaining work: logic-analyzer verification of a continuous multi-frame burst
> (TODO Phase C) — confirm contiguous `AA 55`, advancing `seq`, all five channels and
> CRC-16 across a burst, and keep the link point-to-point (`SPI_NSS_SOFT` on `PB6`).
