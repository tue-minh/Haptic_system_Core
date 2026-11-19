# STM32_FIRMWARE_SLAVE — ITV Valve ADC & SPI1 Slave System

> **MCU**: `STM32F103C8Tx` (`72 MHz` SYSCLK / HCLK, `36 MHz` APB1, `72 MHz` APB2, `12 MHz` ADC)  
> **CubeMX Project**: [STM32_FIRMWARE_SLAVE.ioc](STM32_FIRMWARE_SLAVE.ioc)  
> **Related TODO**: [TODO_SPI_SLAVE.md](TODO_SPI_SLAVE.md)

---

## 1. System Architecture

`STM32_FIRMWARE_SLAVE` samples 5 analog feedback channels from ITV pneumatic valves (`PA0`–`PA4`), applies offset compensation and moving-average filtering, provides a `USART1` debug/plotting CLI, and exposes the 5-channel readings to the `STM32_FIRMWARE` master over `SPI1` Slave DMA.

---

## 2. Peripheral & Pinout Map

| Subsystem | Peripheral | Pins | Configuration | Source |
| :--- | :--- | :--- | :--- | :--- |
| **5-Ch ADC** | `ADC1` + `DMA1_Channel1` | `PA0` (`IN0`) – `PA4` (`IN4`) | 5-channel scan, `55.5` cycles sample time, `10 ms` (`100 Hz`) trigger | [adc.c](Core/Src/adc.c), [main.c](Core/Src/main.c) |
| **SPI1 Slave** | `SPI1` (AFIO Remap + `NOJTAG`) | `PB3` (SCK), `PB4` (MISO), `PB5` (MOSI)<br>`PB6` (`SPI1_NSS`, EXTI rising/falling, Input `PULLUP`) | `SPI_MODE_SLAVE`, 8-bit MSB, Mode 0 (`CPOL=0, CPHA=1Edge`), `SPI_NSS_SOFT`, `EXTI9_5` priority `0` | [spi.c](Core/Src/spi.c), [gpio.c](Core/Src/gpio.c) |
| **SPI1 DMA** | `DMA1_Channel2` (RX)<br>`DMA1_Channel3` (TX) | — | Normal mode, Byte alignment, Memory increment | [spi.c](Core/Src/spi.c) |
| **Debug UART** | `USART1` + `DMA1_Ch4/Ch5` | `PA9` (TX), `PA10` (RX) | `115,200` baud, 8N1, IDLE-line RX DMA | [usart.c](Core/Src/usart.c), [main.c](Core/Src/main.c) |
| **Status LED** | `GPIO_Output` | `PC13` (`led`) | Push-Pull | [gpio.c](Core/Src/gpio.c) |

> **SWJ Remap**: `HAL_MspInit()` in [stm32f1xx_hal_msp.c](Core/Src/stm32f1xx_hal_msp.c) calls `__HAL_AFIO_REMAP_SWJ_NOJTAG()` so `PB3` and `PB4` are freed from JTAG for `SPI1` while keeping SWD (`PA13`/`PA14`) active.
>
> **CubeMX note**: `PB6` is now an EXTI rising/falling input and the NVIC priorities differ from the `.ioc` (which still has `PB6.Signal=GPIO_Input` and all priorities `0`). Mirror these settings in STM32CubeMX before regenerating; the full table is in [SPI_SLAVE_IMPLEMENTATION_REPORT.md §7](SPI_SLAVE_IMPLEMENTATION_REPORT.md#7-stm32cubemx-ioc-configuration-to-mirror).

---

## 3. ADC Signal Processing Pipeline

1. Every `10 ms` (`ADC_SAMPLE_PERIOD_MS`), `HAL_ADC_Start_DMA(&hadc1, adc_dma_buf, 5)` samples `PA0`–`PA4`.
2. In `HAL_ADC_ConvCpltCallback()`:
   - Subtracts the `200 mV` zero-offset (`ADC_OFFSET_COUNTS = 248`, clamped to `0`).
   - Updates an 8-sample per-channel moving average (`adc_ring[5][8]`).
   - Latches filtered counts (`0..4095`) into `adc_latest[5]`, increments `adc_seq`, and marks `spi_adc_dirty = 1`.

---

## 4. SPI1 Slave Frame Protocol (`ItvSpiFrame_t`, 16 Bytes LE)

| Offset | Size | Field | Description |
| :--- | :--- | :--- | :--- |
| `0` | 1 B | `sof0` | `0xAA` |
| `1` | 1 B | `sof1` | `0x55` |
| `2` | 1 B | `seq` | Rolling sample counter (`0..255`) |
| `3` | 1 B | `status` | Bit `0`: `adc_valid`, Bit `1`: `adc_error > 0` |
| `4..13` | 10 B | `adc_raw[5]` | `uint16_t[5]` LE filtered ADC counts (`PA0`–`PA4`) |
| `14..15` | 2 B | `crc16` | CRC-16/CCITT-FALSE (`poly=0x1021`, `init=0xFFFF`) over bytes `[0..13]` |

- `SPI1_PackFrame()` snapshots `adc_latest[0..4]`, `adc_seq`, and `status` atomically and appends `crc16`. It runs in `HAL_ADC_ConvCpltCallback()` and publishes the frame into a two-slot shadow buffer, so the SPI ISR only copies 16 prepared bytes before re-arming.
- `HAL_SPI_TxRxCpltCallback()` increments `spi_tx_count`, copies the front shadow frame into `spi_tx_buf`, and re-arms `HAL_SPI_TransmitReceive_DMA()`.
- `PB6` (`SPI1_NSS`) is an EXTI rising/falling input: the falling edge tracks `spi_cs_active`, clears `OVR` and arms the slave if idle; the rising edge defers an `HAL_SPI_Abort()` + fresh re-arm to the main loop so the next assertion starts at byte 0 (`0xAA 0x55`).

---

## 5. USART1 Debug CLI (`115200` 8N1)

| Command | Action |
| :--- | :--- |
| `help` / `?` | Print command list |
| `start` / `stop` | Enable or pause periodic CSV/text logging (default ON, `50 ms`) |
| `read` | Print one immediate 5-channel reading |
| `rate <ms>` | Set streaming interval (`20..10000 ms`) |
| `stat` | Print ADC, SPI1 (`spi`, `spi_err`, `resync`), and UART counters |
