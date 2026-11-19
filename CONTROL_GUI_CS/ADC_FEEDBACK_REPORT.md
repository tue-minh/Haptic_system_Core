# SPI ADC Feedback — C# / .NET 8 WPF Integration Report

> Adds live visibility of the STM32F103 ITV valve ADC slave (`PA0`–`PA4`) to the C#
> application. Firmware and Python-side details live in
> `STM32_FIRMWARE/SPI_ADC_IMPLEMENTATION_REPORT.md`.

---

## 1. Protocol layer (`ControlGui.Core/TueUsb`)

| File | Change |
| :--- | :--- |
| `Defs.cs` | `CanId.SensorAdcCh0_3 = 0x500`, `SensorAdcCh4Meta = 0x501`, `SensorSpiCtrl = 0x502`, `SensorAdcAll = 0x503` |
| `Payloads.cs` | `SpiAdcFeedback` / `SpiAdcCtrlStatus` records; `Encoders.SensorAdcReq()` / `SensorSpiCtrl()`; `Decoders.SensorAdcCh0_3()` / `SensorAdcCh4Meta()` / `SensorAdcAll()` (5 × 12-bit unpack) / `SensorSpiCtrl()` |
| `TueUsbClient.cs` | `ReadSpiAdc()`, `ConfigureSpiAdc()`, `GetSpiAdcStatus()` + mock-engine responses for all four IDs (sine-wave simulated channels) |

`ReadSpiAdc()` performs two transactions, matching Python: `0x503` for all five
12-bit channels + status, then `0x501` for `seq`, `rx_ok_cnt` and `crc_err_cnt`.

## 2. Worker / telemetry (`ControlGui.Core/Motor`)

| File | Change |
| :--- | :--- |
| `MotorSnapshot.cs` | `AdcAvailable`, `AdcUpdated`, `AdcChannels[5]`, `AdcSeq`, `AdcStatus`, `AdcValid`, `AdcError`, `AdcRxOk`, `AdcCrcErr`, `AdcHz`, `AdcEnabled`, `AdcBusId`, `AdcCsIdx`, `AdcPeriodMs`, `AdcTotalFrames`, `AdcLastError` |
| `MotorWorker.cs` | `ProbeSpiAdc()` at connect; ADC read added to the 15 Hz `PollTelemetry()`; `ConfigureSpiAdc()` / `ReadSpiAdc()` commands; a firmware without `0x500`–`0x503` disables ADC polling without dropping the motor link |

The ADC read shares the existing STM32 COM port and the motor worker thread, so no
extra serial link is opened and the 100 Hz haptic streamer is unaffected.

## 3. UI (`ControlGui.Wpf`)

| File | Change |
| :--- | :--- |
| `Panels/AdcPanel.xaml` / `.xaml.cs` | **New** panel: CH0–CH4 rows with raw count, millivolts and a 0–4095 level bar; status line (`seq`, `status` flags, `ok`, `crc-err`, Hz); Poll checkbox, bus (`SPI1`/`SPI3`), CS (`CS1..3`), period (`10/20/50/100 ms`), **Apply** (CAN `0x502`) and **Read now** |
| `MainWindow.xaml` | Panel added to the left control column, below "Haptic output" |
| `MainWindow.xaml.cs` | Wires Apply/Read to `MotorWorker`, updates the panel every 30 FPS tick and clears it on motor disconnect |

## 4. Tests (`ControlGui.Tests/CoreTests.cs`)

New coverage (17 tests total, all passing):

- `TestSpiAdcPackingAndMockClient` — 5 × 12-bit round-trip, status flags, millivolt
  conversion, mock `GetSpiAdcStatus` / `ReadSpiAdc`.
- `TestSpiAdcConfigureMock` — `0x502` apply/query round-trip.
- `TestMotorWorkerMockLifecycle` — extended to assert the worker probes the ADC, exposes
  all five channels, and applies a new bus/CS/period configuration.

## 5. Verification

```text
dotnet build ControlGui.sln          -> 0 warnings, 0 errors
dotnet test ControlGui.Tests/...     -> Passed: 17, Failed: 0
```

## 6. Usage

1. Connect the STM32 (motor + haptic) COM port.
2. The worker probes `0x502`/`0x503`/`0x501` during the handshake; the **ITV valve ADC**
   panel then shows CH0–CH4 in raw counts, millivolts and level bars.
3. Change `Bus` / `CS` / `Period` and press **Apply** to reconfigure the master's
   background SPI polling (`0x502`).
4. Tick `MOCK_PORT` in the connection panel to exercise the panel without hardware.
