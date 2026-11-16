# STM32_FIRMWARE_SLAVE — Firmware Functions & Hardware Configuration

This document describes the peripheral configuration, pinout, DMA channels, interrupts, and application functions for `STM32_FIRMWARE_SLAVE` (STM32F103xB), designed to read feedback from 5 ITV pneumatic valves via 5 ADC channels and log/debug over USART1.

---

## 1. System & Clock Configuration
- **MCU**: STM32F103xB (`STM32F103XX_FLASH.ld`, `startup_stm32f103xb.s`)
- **System Clock (`SystemClock_Config` in `Core/Src/main.c`)**:
  - **HSE**: Enabled, PLL source = HSE, PLL multiplier = `RCC_PLL_MUL9`
  - **SYSCLK / HCLK**: PLLCLK (72 MHz with 8 MHz HSE)
  - **APB1**: `HCLK / 2` (36 MHz)
  - **APB2**: `HCLK / 1` (72 MHz)
  - **ADC Clock**: `RCC_ADCPCLK2_DIV6` (12 MHz)

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
  - `ScanConvMode`: `ADC_SCAN_ENABLE`
  - `ContinuousConvMode`: `DISABLE` (software-triggered 5-channel scan every `20 ms`)
  - `ExternalTrigConv`: `ADC_SOFTWARE_START`
  - `DataAlign`: `ADC_DATAALIGN_RIGHT` (12-bit: `0..4095`)
  - `NbrOfConversion`: `5`
  - `SamplingTime`: `ADC_SAMPLETIME_55CYCLES_5` (all 5 channels)
  - Calibrated at startup via `HAL_ADCEx_Calibration_Start(&hadc1)`
- **DMA**: `DMA1_Channel1` (`DMA_PERIPH_TO_MEMORY`, `DMA_NORMAL`, HalfWord alignment, `MemInc = ENABLE`, `Priority = LOW`)
- **Interrupts**: `DMA1_Channel1_IRQn` and `ADC1_2_IRQn` enabled (Priority 0, Subpriority 0)

### USART1 (Debugging & Telemetry Logging)
- **Source**: `Core/Src/usart.c` (`MX_USART1_UART_Init`, `HAL_UART_MspInit`)
- **Pins**:
  - `PA9` (`USART1_TX`, Alternate Function Push-Pull, High Speed)
  - `PA10` (`USART1_RX`, Input, No Pull)
- **Configuration**: `115200` baud, 8N1, `UART_MODE_TX_RX`, no flow control
- **DMA**:
  - **TX**: `DMA1_Channel4` (`DMA_MEMORY_TO_PERIPH`, `DMA_NORMAL`, Byte alignment, `MemInc = ENABLE`)
  - **RX**: `DMA1_Channel5` (`DMA_PERIPH_TO_MEMORY`, `DMA_NORMAL`, Byte alignment, `MemInc = ENABLE`)
- **Interrupts**: `DMA1_Channel4_IRQn`, `DMA1_Channel5_IRQn`, and `USART1_IRQn` (including IDLE line detection via `HAL_UARTEx_ReceiveToIdle_DMA`)

---

## 3. Application Functions & Interrupt Callbacks (`Core/Src/main.c`)

### Helper Functions
- `ITV_RawToMillivolts(raw)` — Converts 12-bit ADC count (`0..4095`) to millivolts (`0..3300 mV` assuming 3.3 V reference).
- `UART1_StartRxDMA()` — Arms `HAL_UARTEx_ReceiveToIdle_DMA` on `USART1` and disables the DMA Half-Transfer interrupt so RX events trigger only on IDLE line or full buffer.
- `UART1_SendDMA(msg, wait_timeout_ms)` — Non-blocking (or bounded-wait) DMA TX helper using `HAL_UART_Transmit_DMA`.
- `ITV_LogSample(raw, timestamp_ms, wait_tx)` — Formats and transmits all 5 channels as raw ADC counts and millivolts:
  ```text
  [t=120ms] RAW:1024,2048,512,4095,0 | mV:825,1650,413,3300,0
  ```
- `ITV_ProcessCommand()` — Parses incoming `USART1` debug commands.

### Interrupt Callbacks
- `HAL_ADC_ConvCpltCallback` — Copies the 5 DMA-transferred ADC samples from `adc_dma_buf` to `adc_latest`, increments `adc_sample_count`, sets `adc_data_ready`, and clears `adc_busy`.
- `HAL_ADC_ErrorCallback` — Increments `adc_error_count` and clears `adc_busy` so sampling can recover.
- `HAL_UART_TxCpltCallback` — Increments `uart_tx_count` and clears `uart_tx_busy`.
- `HAL_UARTEx_RxEventCallback` — Copies received command bytes, sets `uart_cmd_ready`, increments `uart_rx_count`, and re-arms `UART1_StartRxDMA()`.
- `HAL_UART_ErrorCallback` — Increments `uart_error_count`, clears `uart_tx_busy`, and re-arms `UART1_StartRxDMA()`.

### Serial Debug Commands (115200 8N1)
| Command | Action |
| :--- | :--- |
| `help` or `?` | Print command list |
| `start` | Enable periodic streaming (default ON, `100 ms`) |
| `stop` | Pause periodic streaming |
| `read` | Print one immediate 5-channel reading |
| `rate <ms>` | Set streaming interval (`20..10000 ms`) |
| `stat` | Print ADC sample, UART TX/RX, and error counters |
