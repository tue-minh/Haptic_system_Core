# TODO: Python Control Library Update for SPI ADC Feedback (`tue_usb`)

> **Target Package**: `STM32_FIRMWARE/python/tue_usb/` (and mirror in `CONTROL_GUI/python_reference/tue_usb/` if kept in sync)  
> **Goal**: Add support for the new SPI ADC feedback USB CAN IDs (`0x500`, `0x501`, `0x502`) to `TueUsbClient`, matching the existing `haptic_ping` / `haptic_uart_diag` pattern.  
> **Status**: Planning / TODO only — do **not** implement code from this document yet.

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
- [ ] **Add `CanId` enum members** under the `0x500–0x57F` sensor range:
  - `SENSOR_ADC_CH0_3 = 0x500`
  - `SENSOR_ADC_CH4_META = 0x501`
  - `SENSOR_SPI_CTRL = 0x502`

### Phase B: `tue_usb/payloads.py`
- [ ] **Add `SpiAdcFeedback` dataclass**:
  - Fields: `channels: Tuple[int, int, int, int, int]`, `seq: int`, `status: int`, `rx_ok_cnt: int`, `crc_err_cnt: int`.
  - Helper properties/methods:
    - `adc_valid -> bool` (`bool(self.status & 0x01)`)
    - `adc_error -> bool` (`bool(self.status & 0x02)`)
    - `millivolts(vref_mv: int = 3300) -> Tuple[float, ...]` (`raw * vref_mv / 4095.0`)
- [ ] **Add `SpiAdcCtrlStatus` dataclass**:
  - Fields: `enabled: bool`, `bus_id: int`, `cs_idx: int`, `period_ms: int`, `total_frames: int`.
- [ ] **Add `Encoders` & `Decoders` methods**:
  - `Encoders.sensor_spi_ctrl(enable: bool, bus_id: int = 1, cs_idx: int = 0, period_ms: int = 10) -> bytes`
  - `Decoders.sensor_adc_ch0_3(payload: bytes) -> Tuple[int, int, int, int]` (`struct.unpack("<4H", payload)`)
  - `Decoders.sensor_adc_ch4_meta(payload: bytes) -> Tuple[int, int, int, int, int]` (`struct.unpack("<HBBHH", payload)`)
  - `Decoders.sensor_spi_ctrl(payload: bytes) -> SpiAdcCtrlStatus` (`struct.unpack("<4BI", payload)`)

### Phase C: `tue_usb/client.py` & `tue_usb/__init__.py`
- [ ] **Add high-level methods on `TueUsbClient`**:
  - `read_spi_adc(self, cs_idx: Optional[int] = None, timeout: Optional[float] = None) -> SpiAdcFeedback`:
    - Transact `CanId.SENSOR_ADC_CH0_3` and `CanId.SENSOR_ADC_CH4_META`, combine into `SpiAdcFeedback`.
  - `configure_spi_adc(self, enable: bool = True, bus_id: int = 1, cs_idx: int = 0, period_ms: int = 10) -> SpiAdcCtrlStatus`:
    - Transact `CanId.SENSOR_SPI_CTRL`.
  - `get_spi_adc_status(self) -> SpiAdcCtrlStatus`:
    - Transact `CanId.SENSOR_SPI_CTRL` with empty payload.
- [ ] **Export new symbols** (`SpiAdcFeedback`, `SpiAdcCtrlStatus`) in `tue_usb/__init__.py`.
- [ ] **Check `CONTROL_GUI/python_reference/tue_usb/`**:
  - Sync the same updates to `CONTROL_GUI/python_reference/tue_usb/` if the GUI uses that copy.

### Phase D: Tests & Example (`STM32_FIRMWARE/python/tests/`, `examples/`)
- [ ] **Add unit tests** for `Encoders`, `Decoders`, `SpiAdcFeedback.millivolts()`, and mocked `TueUsbClient.read_spi_adc()` / `configure_spi_adc()`.
- [ ] **Add a small CLI example** (e.g. `examples/read_spi_adc.py`) that polls `read_spi_adc()` at 20–100 Hz and prints all 5 channels in raw counts and mV.
