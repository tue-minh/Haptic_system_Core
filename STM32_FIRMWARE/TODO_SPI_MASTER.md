# TODO: SPI Master & USB ADC Feedback Implementation (`STM32_FIRMWARE`)

> **Target MCU**: `STM32F407VETx` (`168 MHz` SYSCLK, `APB2 = 84 MHz`, `APB1 = 42 MHz`)  
> **SPI Config**: `SPI1` (`2.625 MHz`, `CPOL=LOW`, `CPHA=1EDGE`, CS: `SPI1_CS1` `PB8`, `SPI1_CS2` `PB9`, `SPI1_CS3` `PE0`) and `SPI3` (`2.625 MHz`, CS: `SPI3_CS1` `PD4`, `SPI3_CS2` `PD3`, `SPI3_CS3` `PD2`)  
> **Goal**: Select a slave chip via CS, continuously read all 5-channel ADC frames (`ItvSpiFrame_t`, 16 bytes) over SPI, and expose new USB CAN IDs (mirroring the UART bridge pattern) so the host PC can read the SPI ADC feedback.  
> **Status**: Implemented — driver in `Core/Src/tue_spi_adc.c`, USB IDs `0x500`–`0x502` in `tue_usb_binary.c`. See [SPI_ADC_IMPLEMENTATION_REPORT.md](SPI_ADC_IMPLEMENTATION_REPORT.md). Hardware bench verification remains.

---

## 1. Slave Wire Frame Format (16 Bytes, Little-Endian)

Must match `ItvSpiFrame_t` in `STM32_FIRMWARE_SLAVE/Core/Src/main.c`:

| Offset | Size | Field | Description |
| :--- | :--- | :--- | :--- |
| `0` | 1 B | `sof0` | `0xAA` |
| `1` | 1 B | `sof1` | `0x55` |
| `2` | 1 B | `seq` | Rolling sequence counter (`0..255`) |
| `3` | 1 B | `status` | Bit `0`: `adc_valid`, Bit `1`: `adc_error` |
| `4..13` | 10 B | `adc_raw[5]` | 5x `uint16_t` LE (`0..4095`, `PA0`–`PA4`) |
| `14..15` | 2 B | `crc16` | CRC-16/CCITT-FALSE (`poly=0x1021`, `init=0xFFFF`) over bytes `[0..13]` |

---

## 2. Proposed USB Binary CAN IDs (`tue_usb_binary.h`)

`tue_usb_binary.h` reserves `0x300–0x3FF` for user custom streams (enforced by `TUE_USB_ASSERT_NOT_RESERVED`) and allocates **`0x500–0x57F` for Sensors / inputs**.  
Because each USB binary frame has `TUE_USB_MAX_DLC = 8` bytes and 5x `uint16_t` ADC values require `10` bytes, split the feedback cleanly across two `0x50x` IDs plus a control/diagnostic ID:

| CAN ID | Constant Name | Direction | Payload (8 Bytes Max) |
| :--- | :--- | :--- | :--- |
| `0x500` | `TUE_USB_CAN_SENSOR_ADC_CH0_3` | Host `REQ` (`DLC=0` or `[cs_idx]`) -> MCU `RESP_OK` (`DLC=8`) | `adc_raw[0]` (`u16`), `adc_raw[1]` (`u16`), `adc_raw[2]` (`u16`), `adc_raw[3]` (`u16`) |
| `0x501` | `TUE_USB_CAN_SENSOR_ADC_CH4_META` | Host `REQ` (`DLC=0` or `[cs_idx]`) -> MCU `RESP_OK` (`DLC=8`) | `adc_raw[4]` (`u16`), `seq` (`u8`), `status` (`u8`), `rx_ok_cnt` (`u16`), `crc_err_cnt` (`u16`) |
| `0x502` | `TUE_USB_CAN_SENSOR_SPI_CTRL` | Host `REQ` (`DLC=4`: `enable`, `bus_id`, `cs_idx`, `period_ms`) -> MCU `RESP_OK` (`DLC=8`) | `[0]` `enabled`, `[1]` `bus_id`, `[2]` `cs_idx`, `[3]` `period_ms`, `[4..7]` `total_frames` (`u32`) |

> Add `TUE_USB_ASSERT_NOT_RESERVED(...)` checks for all three IDs in [tue_usb_binary.h](Core/Inc/tue_usb_binary.h).

---

## 3. TODO List

### Phase A: New SPI ADC Driver (`Core/Inc/tue_spi_adc.h`, `Core/Src/tue_spi_adc.c`)
- [x] **Define CS table & bus abstraction**:
  - Map logical slave indices (e.g. `0..2` on `SPI1`: `PB8`, `PB9`, `PE0`; `3..5` on `SPI3`: `PD4`, `PD3`, `PD2`) to `(SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t pin)`.
- [x] **Implement `tue_spi_adc_init()`**:
  - Verify all CS pins are `GPIO_PIN_SET` (HIGH / idle).
  - Initialize cached frame struct (`adc_raw[5]`, `seq`, `status`, `last_update_tick`, `rx_ok_cnt`, `crc_err_cnt`).
- [x] **Implement continuous CS-selected burst read (`tue_spi_adc_poll()`)**:
  - Assert selected CS LOW (`GPIO_PIN_RESET`), wait a short setup delay for slave EXTI/DMA readiness.
  - Perform `HAL_SPI_TransmitReceive(&hspiX, dummy_tx, rx_buf, 8 x 16, timeout)` while CS stays LOW.
  - Deassert CS HIGH (`GPIO_PIN_SET`).
  - Validate `sof0 == 0xAA`, `sof1 == 0x55`, and `CRC16-CCITT` with a sliding window. On pass, atomically update the cached `adc_raw[5]`, `seq`, `status`, and increment `rx_ok_cnt`; on fail, increment `crc_err_cnt`.
- [x] **CRC-16 helper**:
  - Implement `crc16_ccitt_false()` matching the slave (`poly=0x1021`, `init=0xFFFF`).

### Phase B: FreeRTOS Task Integration (`Core/Src/freertos.c`)
- [x] **Add a periodic SPI polling task** (created by `tue_spi_adc_init()`, started from `tue_usb_init()`):
  - When enabled (`s_enabled == 1`), poll the active CS at the configured period (default `10 ms` / `100 Hz`).
  - Use a PRIMASK critical section when copying the 5-channel `adc_raw[5]` cache for USB requests.

### Phase C: USB Binary Protocol Handlers (`Core/Inc/tue_usb_binary.h`, `Core/Src/tue_usb_binary.c`)
- [x] **Register CAN IDs `0x500`, `0x501`, `0x502`** in `tue_usb_binary.h` with `TUE_USB_ASSERT_NOT_RESERVED`.
- [x] **Implement dispatch cases in `tue_usb_binary.c`**:
  - `0x500` (`TUE_USB_CAN_SENSOR_ADC_CH0_3`): return `RESP_OK` with `adc_raw[0..3]` (`8` bytes).
  - `0x501` (`TUE_USB_CAN_SENSOR_ADC_CH4_META`): return `RESP_OK` with `adc_raw[4]`, `seq`, `status`, `rx_ok_cnt`, `crc_err_cnt` (`8` bytes).
  - `0x502` (`TUE_USB_CAN_SENSOR_SPI_CTRL`): configure active bus/CS/polling rate or query status.
  - [ ] Optional: support `TUE_USB_FRAME_ASYNC` streaming if host-initiated polling (`0x500` + `0x501`) is not fast enough.

### Phase D: Build & Hardware Verification
- [x] Build `STM32_FIRMWARE` cleanly with zero warnings on new `0x500–0x502` static asserts.
- [ ] Verify with `STM32_FIRMWARE_SLAVE` connected on `SPI1` (`PB3/PB4/PB5` + `PB8 -> PB6` CS) that `0x500` / `0x501` return live `PA0`–`PA4` values over USB.
