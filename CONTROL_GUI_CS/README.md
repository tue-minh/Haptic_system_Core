# Unified Motor Control & Dual PaXini Tactile GUI (C# / .NET 8 WPF)

A complete, high-performance C# rewrite of the Unified Motor Control and Dual PaXini Tactile acquisition application, ported from Python/PyQt6 into native **.NET 8 WPF**.

---

## 1. Solution Architecture

```
CONTROL_GUI_CS/
├── ControlGui.sln
├── ControlGui.Core/                   # Core Library (.NET 8)
│   ├── TueUsb/                        # TUE Binary Protocol (CAN-over-USB)
│   │   ├── Defs.cs                    # CAN IDs, error codes, enum flags
│   │   ├── Crc8.cs                    # SAE J1850 CRC-8 lookup table & self-test
│   │   ├── Frame.cs                   # 20-byte wire frame serialization/deserialization
│   │   ├── Payloads.cs                # High-level encoders and decoders for all subsystems
│   │   ├── FrameParser.cs             # Streaming byte-by-byte defensive state machine
│   │   └── TueUsbClient.cs            # Serial client, transact queue, mock simulation engine
│   ├── Paxini/                        # PaXini High-Speed Communication Board Protocol
│   │   ├── PaxiniConstants.cs         # 921600 baud, registers, 0.1 N force LSB
│   │   ├── S2015EliteGeometry.cs      # Exact physical 3D taxel coordinates (52 taxels in mm)
│   │   ├── PaxiniProtocol.cs          # LRC checksums, frame builders, address probing (0x01-0x08)
│   │   ├── SensorMetrics.cs           # Fz, Ft, |F|, friction μ, CoP centroid, tilt, shear angle
│   │   ├── RingBuffer.cs              # High-throughput ring buffer for real-time plot windowing
│   │   ├── PaxiniSensorWorker.cs      # Dedicated thread at 921600 baud, tare accumulation, metrics
│   │   └── SensorLink.cs              # Facade managing worker lifecycle for Left & Right fingers
│   ├── Haptic/                        # 100 Hz Master/Slave Haptic Teleoperation Bridge
│   │   ├── HapticConstants.cs         # CAN 0x300, 5 pneumatic valves (A-E), finger layout
│   │   ├── HapticPayloadSynthesizer.cs# 8-byte payload compression from dual 52-taxel arrays
│   │   ├── TueBinaryEncoder.cs        # 20-byte CAN 0x300 binary frame encoder
│   │   └── Stm32HapticStreamer.cs     # 100 Hz regulated streaming thread (1 ms winmm multimedia timer)
│   ├── Motor/                         # Stepper & Gripper Motion Subsystem
│   │   ├── MotorConstants.cs          # Velocity/accel limits, gripper presets (55 open / 420 close)
│   │   ├── MotorSnapshot.cs           # Thread-safe immutable telemetry snapshot
│   │   ├── LockedSerialPort.cs        # Thread-safe write multiplexing (motor control + 100 Hz haptic frames)
│   │   └── MotorWorker.cs             # Background thread with urgent E-stop queue & 15 Hz telemetry poll
│   ├── Recording/                     # Synchronized Data Recording & High-Speed CSV Logging
│   │   ├── RecordingSchema.cs         # 345-column master schema and 171-column per-sensor schema
│   │   ├── CsvRecorder.cs             # Buffered background writer for individual sensor files
│   │   ├── UnifiedRecorder.cs         # 100 Hz synchronized sampler and batched disk flusher
│   │   └── RecordingManager.cs        # Coordinates master & per-sensor logs, directory selector
│   └── Analysis/                      # Offline Data Analysis & Diagnostic Reporting
│       └── PaxiniDataAnalyzer.cs      # C# port of plot_paxini_data.py for offline CSV parsing
├── ControlGui.Wpf/                    # User Interface (.NET 8 WPF Desktop)
│   ├── Theme.cs                       # Professional palette (monochrome with information-carrying accents)
│   ├── Controls/
│   │   ├── StatusLed.cs               # Hardware status LED (Ready, Connecting, Warning, Error, Off)
│   │   ├── JogButton.cs               # Press-and-hold jog button with safety release watchdog
│   │   ├── TaxelMapView.xaml/.cs      # Live 2D 52-taxel pressure map with CoP marker & shear vector
│   │   ├── ValveMapView.xaml/.cs      # Live 5-valve pneumatic thimble map with commanded force & telemetry
│   │   ├── Tactile3DView.xaml/.cs     # Interactive 3D point cloud with mouse-drag rotation & quiver arrow
│   │   └── ForceHistoryPlot.xaml/.cs  # Real-time 2D scrolling force curves with direction filter
│   ├── Panels/
│   │   ├── ConnectionPanel.xaml/.cs   # 3 COM ports manager, baud selection, auto-detect, collision guard
│   │   ├── StepperPanel.xaml/.cs      # Stepper telemetry (XYZ), driver enable, speed/accel, jog cross, E-stop
│   │   ├── GripperPanel.xaml/.cs      # STS3215 Servo control, bus scan, ping, presets (55 open / 420 close)
│   │   ├── HapticPanel.xaml/.cs       # 100 Hz haptic streaming toggle, CAN 0x300 info, ESP32 UART ping
│   │   ├── RecordingPanel.xaml/.cs    # Master recording controls, elapsed time, row counter, directory picker
│   │   └── SensorMonitorPanel.xaml/.cs# 2D scrolling curves, direction selector, Tare L/R/Both, metrics grid
│   ├── MainWindow.xaml/.cs            # Window layout, 30 FPS DispatcherTimer, safety watchdog
│   └── App.xaml/.cs                   # Application entry point, styling and visual templates
└── ControlGui.Tests/                  # Comprehensive MSTest Suite
    └── CoreTests.cs                   # 12 Unit tests verifying CRC-8, framing, state machine, haptic synthesis,
                                       # LRC checksums, metrics, mock engine, and 345-col recording schemas
```

---

## 2. Key Features

1. **Deterministic Multi-threading & Low Latency**:
   - Zero serial I/O on UI thread.
   - Dedicated threads for motor control (15 Hz poll), tactile sensor acquisition (921,600 baud streaming), and haptic output (100 Hz regulated with 1 ms Windows multimedia timer resolution).
   - Urgent E-stop queue processed immediately before regular commands.

2. **Shared USB CDC Multiplexing (`LockedSerialPort`)**:
   - The STM32 single COM port carries both motor commands and 100 Hz haptic binary frames (CAN ID `0x300`).
   - Writes are synchronized atomically with mutual exclusion locks to prevent frame interleaving.

3. **Sensor Auto-Detection & Tare Baseline**:
   - Automatically probes sensor EEPROM bus addresses (`0x01` through `0x08`).
   - Multi-frame rolling tare zeroing both resultant forces and all 52 individual taxels.

4. **Rich Visualizations**:
   - **2D Taxel Map**: 52 taxels rendered at physical $(x, y)$ coordinates with normal force colormap, Center of Pressure marker `+`, and tangential shear vector.
   - **3D Tactile Surface & Quiver**: 3D point cloud projected with real-time mouse-drag rotation (azimuth & elevation) and 3D resultant force arrow.
   - **Haptic Valve Map**: 5 pneumatic chambers (A/B/C underside pad Tip $\to$ Base, D/E flanks) colored by commanded force with raw wire bytes and age.
   - **Real-time 2D Force History**: High-speed scrolling curves ($Fz, Fx, Fy, |F|, Ft$) with direction filtering.

5. **Single-File SQLite Telemetry Export (Replaces CSV)**:
   - All recordings are saved into a single SQLite database (`recordings.db`).
   - Only 1 SQLite database file is created once on disk.
   - Each recording session creates a new dedicated table named after the session timestamp (`rec_YYYYMMDD_HHMMSS`).
   - High-throughput write engine: WAL mode, prepared statements, and batched transactions easily sustaining 100 Hz sampling across all 345 synchronized channels (Steppers XYZ, Gripper Pos/Volt, Left/Right forces, metrics, and all 52 taxels).
   - Session catalog table `_recording_sessions` logs start time, duration, sample rate, row count, and metadata for fast offline queries.

6. **Integrated Mock Engine**:
   - Select `MOCK_PORT` in the connection panel to run full simulation without physical hardware attached.

---

## 3. How to Build & Run

### Prerequisites
- Windows 10 / 11
- [.NET 8 SDK](https://dotnet.microsoft.com/download/dotnet/8.0)

### Build
Open PowerShell in this directory:
```powershell
dotnet build
```

### Run Tests
```powershell
dotnet test
```

### Launch the GUI
```powershell
dotnet run --project ControlGui.Wpf
```

---

## 4. Hardware COM Port Assignments

| Role | Interface | Default Baud | Description |
|:---|:---|:---|:---|
| **STM32 (Motor + Haptic)** | USB Virtual COM Port | `1,000,000` | CoreXY + Z steppers, STS3215 gripper servo, 100 Hz haptic frames (CAN `0x300`), ESP32 relay (CAN `0x301`) |
| **Left Sensor** | USB-UART Converter | `921,600` | PaXini PX6AX-GEN3 DP-S2015-Elite tactile sensor (52 taxels) |
| **Right Sensor** | USB-UART Converter | `921,600` | PaXini PX6AX-GEN3 DP-S2015-Elite tactile sensor (52 taxels) |
