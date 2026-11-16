# STM32_FIRMWARE_SLAVE — Firmware Functions & Hardware Configuration

This document describes the peripheral configuration, pinout, DMA channels, interrupts, SPI slave protocol, and application functions for `STM32_FIRMWARE_SLAVE` (STM32F103xB), designed to read feedback from 5 ITV pneumatic valves via 5 ADC channels, log/debug over USART1, and stream the 5 ADC values to the master (`STM32_FIRMWARE`) over SPI1 Slave.

---

## 1. System & Clock Configuration
- **MCU**: STM32F103xB (`STM32F103XX_FLASH.ld`, `startup_stm32f103xb.s`)
- **System Clock (`SystemClock_Config` in `Core/Src/main.c`)**:
  - **HSE**: Enabled, PLL source = HSE, PLL multiplier = `RCC_PLL_MUL9`
  - **SYSCLK / HCLK**: PLLCLK (72 MHz with 8 MHz HSE)
  - **APB1**: `HCLK / 2` (36 MHz)
  - **APB2**: `HCLK / 1` (72 MHz)
  - **ADC Clock**: `RCC_ADCPCLK2_DIV6` (12 MHz)
- **Debug / JTAG Remap (`HAL_MspInit` in `Core/Src/stm32f1xx_hal_msp.c`)**:
  - `__HAL_AFIO_REMAP_SWJ_NOJTAG()` frees `PB3` and `PB4` for `SPI1` while keeping SWD active.

---

## 2. Peripheral & Pinout Summary

### ADC1 (5-Channel ITV Valve Feedback Reading)
- **Source**: `Core/Src/adc.c` (`MX_ADC1_Init`, `HAL_ADC_MspInit`)
- **Pins**:
  - `PA0` (`ADC1_IN0`, Rank 1)
  - `PA1` (`ADC1_IN1`, Rank 2)
  - `PA2` (`ADC1_IN2`, Rank 3)
  - `PA3` (`ADC1_IN3`, Rank 4)
  - `PA4` (`ADC1_IN4`, Rank 5)
- **Configuration**:
  - `ScanConvMode`: `ADC_SCAN_ENABLE`, `ContinuousConvMode`: `DISABLE`, `NbrOfConversion`: `5`, `SamplingTime`: `ADC_SAMPLETIME_55CYCLES_5`
  - Calibrated at startup via `HAL_ADCEx_Calibration_Start(&hadc1)`, triggered every `20 ms`
- **DMA & Interrupts**: `DMA1_Channel1` (`DMA_PERIPH_TO_MEMORY`, HalfWord), `DMA1_Channel1_IRQn`, `ADC1_2_IRQn`

### SPI1 (Slave Interface to Master `STM32_FIRMWARE`)
- **Source**: `Core/Src/spi.c` (`MX_SPI1_Init`, `HAL_SPI_MspInit`)
- **Pins (AFIO SPI1 Remap enabled)**:
  - `PB3` (`SPI1_SCK`, Input from Master)
  - `PB4` (`SPI1_MISO`, AF Push-Pull output to Master)
  - `PB5` (`SPI1_MOSI`, Input from Master)
  - `PB6` (`SPI1_NSS_Pin`, GPIO Input active-low CS from Master)
- **Configuration** (matches Master `STM32_FIRMWARE` SPI1 settings):
  - `Mode`: `SPI_MODE_SLAVE`, `Direction`: `SPI_DIRECTION_2LINES`
  - `DataSize`: `SPI_DATASIZE_8BIT`, `FirstBit`: `SPI_FIRSTBIT_MSB`
  - `CLKPolarity`: `SPI_POLARITY_LOW` (CPOL = 0), `CLKPhase`: `SPI_PHASE_1EDGE` (CPHA = 0)
  - `NSS`: `SPI_NSS_SOFT` (CS monitored on `PB6`)
- **DMA & Interrupts**:
  - **RX**: `DMA1_Channel2` (`DMA_PERIPH_TO_MEMORY`, Byte, `DMA_PRIORITY_HIGH`)
  - **TX**: `DMA1_Channel3` (`DMA_MEMORY_TO_PERIPH`, Byte, `DMA_PRIORITY_HIGH`)
  - **Interrupts**: `DMA1_Channel2_IRQn`, `DMA1_Channel3_IRQn`, `SPI1_IRQn`

### USART1 (Debugging & Telemetry Logging)
- **Source**: `Core/Src/usart.c` (`MX_USART1_UART_Init`, `HAL_UART_MspInit`)
- **Pins**: `PA9` (`USART1_TX`), `PA10` (`USART1_RX`)
- **Configuration**: `115200` baud, 8N1, `UART_MODE_TX_RX`
- **DMA & Interrupts**: `DMA1_Channel4` (TX), `DMA1_Channel5` (RX), `USART1_IRQn` (IDLE line via `HAL_UARTEx_ReceiveToIdle_DMA`)

---

## 3. SPI1 Slave Packet Format (`ItvSpiFrame_t`, 16 Bytes)

To read all 5 valve ADC values from the master (`STM32_FIRMWARE`), pull `PB6` (`CS`) LOW, clock **16 bytes** on `SPI1` (Mode 0, 8-bit MSB first), and pull `PB6` HIGH:

| Byte Offset | Size | Field | Description |
| :--- | :--- | :--- | :--- |
| `0` | 1 | `sof0` | Sync byte 0 (`0xAA`) |
| `1` | 1 | `sof1` | Sync byte 1 (`0x55`) |
| `2` | 1 | `seq` | Rolling sample counter (`0..255`, increments each ADC scan) |
| `3` | 1 | `status` | Bit 0: `1` = ADC data valid; Bit 1: `1` = ADC error occurred |
| `4..13` | 10 | `adc_raw[5]` | `uint16_t[5]` little-endian raw 12-bit ADC counts (`PA0..PA4`, `0..4095`) |
| `14..15` | 2 | `crc16` | `uint16_t` little-endian CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) over bytes `0..13` |

---

## 4. Application Functions & Interrupt Callbacks (`Core/Src/main.c`)

### Helper Functions
- `SPI1_CalcCRC16(data, len)` — Computes CRC-16/CCITT-FALSE (`0x1021`, init `0xFFFF`).
- `SPI1_PackFrame(buf)` — Atomically snapshots `adc_latest[0..4]`, `adc_seq`, and status into `ItvSpiFrame_t` with CRC-16.
- `SPI1_RearmSlaveDMA()` — Resets/aborts SPI1 if needed, clears `OVR`, packs the latest ADC frame into `spi_tx_buf`, and arms `HAL_SPI_TransmitReceive_DMA`.
- `ITV_RawToMillivolts(raw)` — Converts 12-bit ADC count (`0..4095`) to millivolts (`0..3300 mV`).
- `UART1_StartRxDMA()`, `UART1_SendDMA()`, `ITV_LogSample()`, `ITV_ProcessCommand()` — Non-blocking USART1 DMA logging and debug command handling.

### Interrupt Callbacks
- `HAL_ADC_ConvCpltCallback` — Latches `adc_dma_buf[0..4]` into `adc_latest`, increments `adc_seq`/`adc_sample_count`, and marks `spi_adc_dirty = 1`.
- `HAL_SPI_TxRxCpltCallback` — Increments `spi_tx_count`, packs the latest ADC frame, and immediately re-arms `HAL_SPI_TransmitReceive_DMA`.
- `HAL_SPI_ErrorCallback` — Increments `spi_error_count`, clears `OVR`, and re-arms DMA if SPI1 is ready (otherwise main loop recovers when `PB6` CS is idle).
- `HAL_UART_TxCpltCallback`, `HAL_UARTEx_RxEventCallback`, `HAL_UART_ErrorCallback` — Manage USART1 DMA TX/RX and IDLE command reception.

### Serial Debug Commands (`USART1`, 115200 8N1)
| Command | Action |
| :--- | :--- |
| `help` or `?` | Print command list |
| `start` | Enable periodic streaming (default ON, `100 ms`) |
| `stop` | Pause periodic streaming |
| `read` | Print one immediate 5-channel reading |
| `rate <ms>` | Set streaming interval (`20..10000 ms`) |
| `stat` | Print ADC, SPI1 (`spi`, `spi_err`, `resync`), and UART counters |
