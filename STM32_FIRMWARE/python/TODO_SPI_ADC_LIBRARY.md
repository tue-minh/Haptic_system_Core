# TODO: Python Control Library Update for SPI ADC Feedback (`tue_usb`)

> **Target Package**: `STM32_FIRMWARE/python/tue_usb/` (and mirror in `CONTROL_GUI/python_reference/tue_usb/` if kept in sync)  
> **Goal**: Add support for the new SPI ADC feedback USB CAN IDs (`0x500`, `0x501`, `0x502`) to `TueUsbClient`, matching the existing `haptic_ping` / `haptic_uart_diag` pattern.  
> **Status**: Implemented — `tue_usb` (both copies) and the GUI (ADC panel + `MotorWorker` polling) support `0x500`–`0x503`. Example: `examples/read_spi_adc.py`.

---

## 1. Protocol Mapping (`0x500`–`0x502`)

| `CanId` Member | Value | Request Payload | Response Payload (8 Bytes LE) |
| :--- | :--- | :--- | :--- |
| `SENSOR_ADC_CH0_3` | `0x500` | `0` bytes (or `1` byte `cs_idx`) | `<4H` -> `ch0, ch1, ch2, ch3` (`uint16_t` raw `0..4095`) |
| `SENSOR_ADC_CH4_META` | `0x501` | `0` bytes (or `1` byte `cs_idx`) | `<HBBHH` -> `ch4`, `seq`, `status`, `rx_ok_cnt`, `crc_err_cnt` |
| `SENSOR_SPI_CTRL` | `0x502` | `<4B` (`enable, bus_id, cs_idx, period_ms`) or `0` bytes query | `<4BI` -> `enabled, bus_id, cs_idx, period_ms, total_frames` |

---

## 2. TODO List

### Phase A: `tue_usb/defs.py`
- [x] **Add `CanId` enum members** under the `0x500–0x57F` sensor range:
  - `SENSOR_ADC_CH0_3 = 0x500`
  - `SENSOR_ADC_CH4_META = 0x501`
  - `SENSOR_SPI_CTRL = 0x502`
  - `SENSOR_ADC_ALL = 0x503` (added: all five channels in one 8-byte frame)

### Phase B: `tue_usb/payloads.py`
- [x] **Add `SpiAdcFeedback` dataclass**:
  - Fields: `channels: Tuple[int, int, int, int, int]`, `seq: int`, `status: int`, `rx_ok_cnt: int`, `crc_err_cnt: int`.
  - Helper properties/methods:
    - `adc_valid -> bool` (`bool(self.status & 0x01)`)
    - `adc_error -> bool` (`bool(self.status & 0x02)`)
    - `millivolts(vref_mv: int = 3300) -> Tuple[float, ...]` (`raw * vref_mv / 4095.0`)
- [x] **Add `SpiAdcCtrlStatus` dataclass**:
  - Fields: `enabled: bool`, `bus_id: int`, `cs_idx: int`, `period_ms: int`, `total_frames: int`.
- [x] **Add `Encoders` & `Decoders` methods**:
  - `Encoders.sensor_adc_req(cs_idx: Optional[int] = None) -> bytes`
  - `Encoders.sensor_spi_ctrl(enable: bool, bus_id: int = 1, cs_idx: int = 0, period_ms: int = 10) -> bytes`
  - `Decoders.sensor_adc_ch0_3(payload: bytes) -> Tuple[int, int, int, int]` (`struct.unpack("<4H", payload)`)
  - `Decoders.sensor_adc_ch4_meta(payload: bytes) -> Tuple[int, int, int, int, int]` (`struct.unpack("<HBBHH", payload)`)
  - `Decoders.sensor_adc_all(payload: bytes) -> (channels, status)` (5 × 12-bit packed + status nibble)
  - `Decoders.sensor_spi_ctrl(payload: bytes) -> SpiAdcCtrlStatus` (`struct.unpack("<BBBBI", payload)`)

### Phase C: `tue_usb/client.py` & `tue_usb/__init__.py`
- [x] **Add high-level methods on `TueUsbClient`**:
  - `read_spi_adc(cs_idx=None, timeout=None) -> SpiAdcFeedback` (0x503 channels + 0x501 sequence/counters).
  - `configure_spi_adc(enable=True, bus_id=1, cs_idx=0, period_ms=10) -> SpiAdcCtrlStatus`.
  - `get_spi_adc_status() -> SpiAdcCtrlStatus`.
- [x] **Add mock-engine support** for all four IDs so the GUI runs without hardware.
- [x] **Export new symbols** (`SpiAdcFeedback`, `SpiAdcCtrlStatus`) in `tue_usb/__init__.py`.
- [x] **Sync the same updates** to `CONTROL_GUI/python_reference/tue_usb/` (the copy the GUI imports).

### Phase D: Tests, Example and GUI (`python/tests/`, `examples/`, `CONTROL_GUI/`)
- [x] **Add unit tests** for `Encoders`, `Decoders`, `SpiAdcFeedback.millivolts()`, and the mocked client (`CONTROL_GUI/tests/test_adc.py`).
- [x] **Add a small CLI example** (`examples/read_spi_adc.py`) that polls at 10–20 Hz and prints all 5 channels in raw counts and mV.
- [x] **GUI**: `unified_gui/panels/adc.py` shows the 5 channels, raw counts, mV, bars, `seq`/status/counters and lets the user change bus/CS/period (CAN 0x502). `MotorWorker` probes the ADC at connect, polls it with the motor telemetry and exposes it in `MotorSnapshot`.
