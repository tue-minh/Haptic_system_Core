# SPI ADC Feedback Implementation Report (`STM32_FIRMWARE` master)

> Completes [TODO_SPI_MASTER.md](TODO_SPI_MASTER.md). Adds a background SPI master that
> polls the `STM32_FIRMWARE_SLAVE` (STM32F103, `ItvSpiFrame_t` 16-byte frames) and exposes
> the five ITV valve ADC channels to the host over three new USB binary CAN IDs.

---

## 1. What was added

| Area | Change |
| :--- | :--- |
| New driver | `Core/Inc/tue_spi_adc.h`, `Core/Src/tue_spi_adc.c` |
| USB protocol | CAN IDs `0x500`, `0x501`, `0x502` registered in `Core/Inc/tue_usb_binary.h` with `TUE_USB_ASSERT_NOT_RESERVED` |
| USB dispatch | `spi_adc_cmd()` handler + dispatch branch in `Core/Src/tue_usb_binary.c` |
| Startup | `tue_spi_adc_init()` called from `tue_usb_init()` (FreeRTOS already running) |
| Build | `Core/Src/tue_spi_adc.c` added to `CMakeLists.txt` |
| Docs | `TODO_SPI_MASTER.md` status/checkboxes, `FIRMWARE_SYSTEM.md` CAN map and peripheral row |

## 2. Driver design (`tue_spi_adc.c`)

- **Slave table** — six logical indices mapped to bus + chip-select:

  | Index | Bus | CS pin | Descriptor |
  | :--- | :--- | :--- | :--- |
  | 0 | SPI1 | `PB8` (`SPI1_CS1`) | `bus_id = 1`, `cs_idx = 0` |
  | 1 | SPI1 | `PB9` (`SPI1_CS2`) | `bus_id = 1`, `cs_idx = 1` |
  | 2 | SPI1 | `PE0` (`SPI1_CS3`) | `bus_id = 1`, `cs_idx = 2` |
  | 3 | SPI3 | `PD4` (`SPI3_CS1`) | `bus_id = 3`, `cs_idx = 0` |
  | 4 | SPI3 | `PD3` (`SPI3_CS2`) | `bus_id = 3`, `cs_idx = 1` |
  | 5 | SPI3 | `PD2` (`SPI3_CS3`) | `bus_id = 3`, `cs_idx = 2` |

- **Poll cycle** (`poll_slave()`), default 100 Hz:
  1. Drive the selected CS LOW and wait a short setup delay (`cs_setup_delay()`, ~200 NOPs).
  2. `HAL_SPI_TransmitReceive()` a burst of `TUE_SPI_ADC_BURST_FRAMES = 8` frames
     (`128` bytes) with `SPI1`/`SPI3` already configured at `2.625 MHz`, mode 0, 8-bit MSB.
  3. Drive CS HIGH and scan the burst with a **sliding window** that requires `SOF = AA 55`
     and `CRC-16/CCITT-FALSE` over bytes `0..13`; the last valid frame wins.
  4. Publish `adc_raw[5]`, `seq`, `status`, `last_update_ms` under a `PRIMASK` critical
     section and increment `rx_ok_cnt`; a failed/CRC-invalid burst increments `crc_err_cnt`.

- **CRC** — `crc16_ccitt_false()` (`poly = 0x1021`, `init = 0xFFFF`) matches the slave.
- **Background task** — `tue_spi_adc_init()` creates the FreeRTOS thread `tue_spi_adc`
  (`osPriorityNormal`, 2048-byte stack). It calls `osDelay(period_ms)` between polls and
  idles at 10 ms when disabled.
- **Concurrency** — the polling task is the only SPI bus user; cached snapshots are read by
  the USB task through a short PRIMASK critical section (no mutex blocking).

## 3. New USB CAN IDs (`tue_usb_binary.h` / `.c`)

| CAN ID | Constant | Request | Response (8 bytes, LE) |
| :--- | :--- | :--- | :--- |
| `0x500` | `TUE_USB_CAN_SENSOR_ADC_CH0_3` | `DLC=0` (active slave) or `[logical_idx]` | `adc_raw[0..3]` as `u16` |
| `0x501` | `TUE_USB_CAN_SENSOR_ADC_CH4_META` | `DLC=0` (active slave) or `[logical_idx]` | `adc_raw[4]` (`u16`), `seq` (`u8`), `status` (`u8`), `rx_ok_cnt` (`u16`), `crc_err_cnt` (`u16`) |
| `0x502` | `TUE_USB_CAN_SENSOR_SPI_CTRL` | `DLC=0` query, or `[enabled, bus_id, cs_idx, period_ms]` | `[enabled, bus_id, cs_idx, period_ms, total_frames(u32)]` |

- `bus_id` is `1` (SPI1) or `3` (SPI3); `cs_idx` is `0..2`; `period_ms` is `1..255`.
- Logical index = `bus_id == 1 ? cs_idx : 3 + cs_idx`.
- Defaults at boot: enabled, `bus_id = 1`, `cs_idx = 0` (SPI1 CS1, `PB8`), `period_ms = 10`.
- `status` bit `0` = `adc_valid`, bit `1` = slave ADC error count non-zero.

## 4. Build verification

`arm-none-eabi-gcc 15.2.1` + CMake/Ninja, Debug:

```text
FLASH:  95,308 B / 512 KB (18.18%)
RAM:    32,632 B / 128 KB (24.90%)
```

Both new `_Static_assert`s for `0x500`–`0x502` pass at compile time.

## 5. Bench wiring / verification

- Connect master `PB8` (`SPI1_CS1`) to slave `PB6` (`SPI1_NSS`).
- `SPI1_SCK` `PB3` ↔ slave `PB3`, `SPI1_MISO` `PB4` ↔ slave `PB4`,
  `SPI1_MOSI` `PB5` ↔ slave `PB5`, plus common GND.
- Sequence:
  1. Flash both firmwares.
  2. `0x502` query (`DLC=0`) must report `enabled=1, bus_id=1, cs_idx=0, period_ms=10`.
  3. `0x500`/`0x501` should return live `PA0`–`PA4` counts and a `seq` that advances
     at ~100 Hz.
  4. `0x501` `rx_ok_cnt` must increase while `crc_err_cnt` stays small; use `0x502` to
     change the slot/rate if needed.

> Remaining work: physical bench validation (TODO Phase D) and the optional ASYNC
> streaming path. The host Python library update is intentionally **not** part of this
> change — `python/TODO_SPI_ADC_LIBRARY.md` explicitly defers it.
