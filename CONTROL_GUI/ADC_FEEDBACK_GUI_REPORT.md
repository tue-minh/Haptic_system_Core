# SPI ADC Feedback — Host Library & GUI Integration Report

> Adds live visibility of the STM32F103 ITV valve ADC slave (`PA0`–`PA4`) to the
> Python library and the unified GUI. Firmware side is documented in
> `STM32_FIRMWARE/SPI_ADC_IMPLEMENTATION_REPORT.md`.

---

## 1. Python library (`tue_usb`)

Updated in **both** copies:

- `STM32_FIRMWARE/python/tue_usb/`
- `CONTROL_GUI/python_reference/tue_usb/` (the copy the GUI imports via `_paths.py`)

| File | Change |
| :--- | :--- |
| `defs.py` | `CanId.SENSOR_ADC_CH0_3 = 0x500`, `SENSOR_ADC_CH4_META = 0x501`, `SENSOR_SPI_CTRL = 0x502`, `SENSOR_ADC_ALL = 0x503` |
| `payloads.py` | `SpiAdcFeedback`, `SpiAdcCtrlStatus`; `Encoders.sensor_adc_req()`, `Encoders.sensor_spi_ctrl()`; `Decoders.sensor_adc_ch0_3()`, `sensor_adc_ch4_meta()`, `sensor_adc_all()`, `sensor_spi_ctrl()` |
| `client.py` | `read_spi_adc()`, `configure_spi_adc()`, `get_spi_adc_status()` + mock-engine responses for all four IDs |
| `__init__.py` | exports the new dataclasses |
| `examples/read_spi_adc.py` | 10–20 Hz console poller (raw counts + mV) |

`read_spi_adc()` issues **two transactions**: `0x503` for all five 12-bit channels and
status, then `0x501` for `seq`, `rx_ok_cnt` and `crc_err_cnt`. The 12-bit packing is
documented in the firmware report §3.1.

## 2. GUI integration

| File | Change |
| :--- | :--- |
| `unified_gui/panels/adc.py` | **New** `AdcPanel`: CH0–CH4 rows with raw count, millivolts and a 0–4095 level bar; status line (`seq`, `status` flags, `ok`, `crc-err`, Hz); Poll checkbox, bus (`SPI1`/`SPI3`), CS (`CS1..3`) and period (`10/20/50/100 ms`) with **Apply** (CAN 0x502) and **Read now** |
| `unified_gui/motor.py` | `MotorSnapshot` gains `adc_*` fields; `_probe_spi_adc()` runs at connect; `_poll()` reads the ADC with the motor telemetry (≈15 Hz); `configure_spi_adc()` / `read_spi_adc()` commands |
| `unified_gui/main_window.py` | Creates the panel in the left column, wires Apply/Read, refreshes it on every tick and clears it on disconnect |
| `tests/test_adc.py` | Codec round-trip, `millivolts()`, panel rendering, Apply signal, unavailable state and a `MotorWorker` integration test with a fake ADC-capable board |

The panel is read-only until the STM32 link is connected; a firmware without
`0x500`–`0x503` degrades cleanly to "not available" (the probe error is logged, the
motor link stays up).

## 3. Verification

- Mock-mode library round-trip:

  ```text
  ctrl: SpiAdcCtrl(enabled=1, bus=SPI1, cs=0, period=10ms, frames=0)
  fb:   SpiAdc[1178, 2742, 3780, 3507, 2129] seq=2 status=0x01 ok=2 crc_err=0
  mV:   [949.3, 2209.7, 3046.2, 2826.2, 1715.7]
  ```

- `python -m unittest tests.test_adc -v` → **6/6 pass**.
- Full GUI suite: **82/83 pass**. The single failure
  (`test_directional_x_and_y_mapping` in `tests/test_haptic.py`) is pre-existing and
  unrelated to this change (it asserts a different synthesizer byte layout).

## 4. Usage

```bash
# library / console
python STM32_FIRMWARE/python/examples/read_spi_adc.py COM3 20

# GUI
python CONTROL_GUI/run_unified_gui.py
# connect the STM32 (motor + haptic) port -> "ITV valve ADC (STM32F103 SPI slave)"
```

The GUI shares the one STM32 COM port already used for motor control and the haptic
stream; no extra port is opened.
