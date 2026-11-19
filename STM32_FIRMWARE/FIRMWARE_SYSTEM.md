# STM32_FIRMWARE — Master Controller System Overview (`MM_Control`)

> **MCU**: `STM32F407VETx` (Cortex-M4, `168 MHz` SYSCLK / HCLK, `42 MHz` APB1, `84 MHz` APB2)  
> **RTOS**: FreeRTOS (CMSIS-RTOS v2)  
> **CubeMX Project**: [MM_control.ioc](MM_control.ioc)  
> **Related TODOs**: [TODO_SPI_MASTER.md](TODO_SPI_MASTER.md) · [python/TODO_SPI_ADC_LIBRARY.md](python/TODO_SPI_ADC_LIBRARY.md)

---

## 1. System Architecture

`STM32_FIRMWARE` is the central motion, actuator, and communication controller (`MM_Control`) for the robot/teleoperation system. It bridges the Host PC (via USB CDC binary CAN-over-USB frames) to:
- **3-Axis Stepper Motors** (CoreXY X/Y + Z tool axis)
- **4x DC Motors** (DRV8872 H-bridge drivers with fault inputs)
- **Feetech STS3215 Smart Bus Servos** (half-duplex 1 Mbps UART)
- **ESP32 Haptic Slave Bridge** (`USART2` @ 115200 baud)
- **STM32F103 ADC Slave(s)** (`SPI1` / `SPI3` @ 2.625 MHz — see [TODO_SPI_MASTER.md](TODO_SPI_MASTER.md))
- **Auxiliary Buses**: `CAN1` (`PD0`/`PD1`), `I2C1` (`PB6`/`PB7`)

---

## 2. Peripheral & Pinout Map

| Subsystem | Peripheral | Pins | Configuration | Firmware Module |
| :--- | :--- | :--- | :--- | :--- |
| **Host Link** | `USB_OTG_FS` | `PA11` (DM), `PA12` (DP) | USB CDC Virtual COM Port | `tue_usb_binary.c/.h`, `usbd_cdc_if.c` |
| **STS3215 Servos** | `USART1` | `PA9` (`SERVO_TX`, Half-Duplex OD) | `1,000,000` baud, 8N1, DMA TX/RX | `tue_sts3215*.c/.h` |
| **Haptic UART Bridge** | `USART2` | `PD5` (TX), `PD6` (RX) | `115,200` baud, 8N1 | `tue_haptic_bridge.c/.h` |
| **SPI1 Master** | `SPI1` | `PB3` (SCK), `PB4` (MISO), `PB5` (MOSI) | `2.625 MHz` (`APB2/32`), Mode 0, 8-bit MSB | `spi.c` |
| **SPI1 Chip Selects** | `GPIO_Output` | `PB8` (`SPI1_CS1`), `PB9` (`SPI1_CS2`), `PE0` (`SPI1_CS3`) | Init `GPIO_PIN_SET` (idle HIGH) | `gpio.c` |
| **SPI3 Master** | `SPI3` | `PC10` (SCK), `PC11` (MISO), `PC12` (MOSI) | `2.625 MHz` (`APB1/16`), Mode 0, 8-bit MSB | `spi.c` |
| **SPI3 Chip Selects** | `GPIO_Output` | `PD4` (`SPI3_CS1`), `PD3` (`SPI3_CS2`), `PD2` (`SPI3_CS3`) | Init `GPIO_PIN_SET` (idle HIGH) | `gpio.c` |
| **Stepper Motors (3ch)** | `TIM1` (1 µs tick) + GPIO | Step: `PE9`, `PE11`, `PE13`<br>Dir: `PC3`, `PC2`, `PC1`, En: `PC5` | `tue_stepper_init()` converts STEP pins to GPIO | `tue_stepper.c/.h` |
| **DC Motors (4ch)** | `TIM3` (CH1–4), `TIM4` (CH1–4) | `PC6–PC9` (DC3/4), `PD12–PD15` (DC1/2) | Dual-PWM DRV8872 drive | `tue_dc.c/.h` |
| **DC Fault Inputs** | `GPIO_Input` | `PD8` (`DC_F1`) – `PD11` (`DC_F4`) | Input with `GPIO_PULLUP` (active LOW fault) | `gpio.c`, `tue_dc.c` |
| **CAN / I2C / LED** | `CAN1`, `I2C1`, `GPIO` | `PD0/PD1` (CAN), `PB6/PB7` (I2C), `PE5` (`LED_MCU`) | `CAN1` normal, `I2C1` 100 kHz | `can.c`, `i2c.c`, `gpio.c` |

---

## 3. USB Binary Protocol (`tue_usb_binary`)

All host-to-MCU communication uses fixed-envelope CAN-over-USB frames over USB CDC (`12` to `20` bytes):

```text
[ SOF1:0xAA | SOF2:0x55 | FRAME_TYPE:1B | CAN_ID:4B LE | DLC:1B | SEQ:1B | PAYLOAD:0..8B | CRC8:1B | EOF1:0x55 | EOF2:0xAA ]
```

- **Integrity**: SAE J1850 CRC-8 over `[FRAME_TYPE .. PAYLOAD]`.
- **CAN ID Allocation**:
  - `0x100–0x107`: System management (`PING`, `INFO`, `UID`, `UPTIME`, `LED_CTRL`, `RESET`, `ECHO`)
  - `0x200–0x21A`: CoreXY + Z stepper motion (`GET_INFO`, `GET_STATUS`, `ENABLE`, `MOVE_*`, `JOG_*`, `STOP`, `ABORT`, `WAIT_IDLE`)
  - `0x280–0x281`: Raw per-motor stepping
  - `0x300–0x3FF`: **Reserved user custom range** (`0x300` Haptic Telemetry -> `USART2`, `0x301` Haptic Ping, `0x302` USART2 Link Diagnostics)
  - `0x400–0x415`: STS3215 bus servos (`PING`, `SET_POS`, `GET_POS`, `SET_SPEED`, `TORQUE`, `MODE`, `PWM`, `FB_*`, `MEM_*`)
  - `0x500–0x57F`: Sensors / inputs (reserved for SPI ADC feedback — see [TODO_SPI_MASTER.md](TODO_SPI_MASTER.md))
  - `0x600–0x607`: DC motors (`GET_INFO`, `SET_SPEED`, `GET_STATUS`, `SET_SPEED_ALL`, `BRAKE`, `COAST`, `GET_FAULTS`, `CONFIG_FREQ`)

---

## 4. FreeRTOS & Startup Flow

1. `main()` ([Core/Src/main.c](Core/Src/main.c)) configures `168 MHz` system clock and initializes `GPIO`, `DMA`, `TIM1/3/4`, `USART1/2`, `CAN1`, `I2C1`, `SPI1`, `SPI3`.
2. `tue_stepper_init()` and `tue_dc_init()` initialize actuator drivers before the scheduler starts.
3. `MX_FREERTOS_Init()` + `osKernelStart()` launch `StartDefaultTask` ([Core/Src/freertos.c](Core/Src/freertos.c)), which initializes USB Device (`MX_USB_DEVICE_Init()`) and starts `tue_usb_init()`.
4. Host Python library lives in [python/tue_usb/](python/tue_usb/) (`TueUsbClient`, `CanId`, `Encoders`, `Decoders`).
