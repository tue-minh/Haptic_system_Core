# Project TODO: Unified PyQt6 GUI for Motor Control & Dual PaXini Tactile Sensors

This document provides a comprehensive, phased implementation roadmap and architectural plan for developing the unified PyQt6 control and acquisition application. The GUI integrates multi-axis stepper motion, smart servo gripper control, and high-speed dual tactile sensor acquisition into a single responsive, multi-threaded interface.

---

## 1. System Architecture & Specifications

```
+---------------------------------------------------------------------------------------+
|                                    PyQt6 Main Window                                  |
|  +---------------------------+  +---------------------------------------------------+ |
|  |     Connection Panel      |  |                Tactile Sensor Section             | |
|  | - Motor COM (1000000)      |  | - Direction Select: [ ] Fx  [ ] Fy  [x] Fz        | |
|  | - Sensor L COM (1000000)  |  | - Realtime 2D Scrolling Curves (pyqtgraph)        | |
|  | - Sensor R COM (1000000)  |  | - 3D Point Cloud Scatter (52 Taxels) + 3D Quiver  | |
|  | [Connect All] [Disconnect]|  | - [Tare Left] [Tare Right] [Tare Both]            | |
|  +---------------------------+  +---------------------------------------------------+ |
|  +---------------------------+  +---------------------------------------------------+ |
|  |    Motor Control Section  |  |             Data Recording Section                | |
|  | [CoreXY + Z Steppers]     |  | [  REC START / STOP  ] Elapsed: 00:00:00          | |
|  | - Pos: X:0.0 Y:0.0 Z:0.0  |  | Logged: Motor State + L/R Forces + 52-Taxel Arrays| |
|  | - Jog: [X+][X-][Y+][Y-]   |  | Background Threaded Queue -> Flush to CSV         | |
|  |        [Z+][Z-] (Press/Rel|  +---------------------------------------------------+ |
|  | - Speed / Accel Sliders   |                                                        |
|  | - [Enable / Disable]      |                                                        |
|  | [STS3215 Gripper Servo]   |                                                        |
|  | - Pos / Voltage Readouts  |                                                        |
|  | - [Open (55) / Close(420)]|                                                        |
|  +---------------------------+                                                        |
+---------------------------------------------------------------------------------------+
```

### 1.1 Hardware Interfaces & COM Port Assignments
1. **Motor Controller (TUE ACSI Board - STM32F407VET6)**
   - **Port**: USB Virtual COM Port (e.g. `COMX`).
   - **Baudrate**: `1000000` bps (8-N-1).
   - **Protocol**: ASCII Token frames (`<TARGET> <ID> <DIR> <FIELD> [ARGS...]`).
   - **Subsystems Controlled**:
     - 3-Axis Stepper System (Motors 1 & 2: CoreXY kinematics; Motor 3: Lead screw Z axis).
     - Feetech STS3215 Smart Bus Servo (Gripper actuator, Servo ID = 1).
2. **Left Tactile Sensor (PaXini PX6AX-GEN3 DP-S2015-Elite)**
   - **Port**: Dedicated USB-UART Board.
   - **Baudrate**: `1000000` bps (1M Baud).
   - **Array Layout**: 52 3D taxels with predefined spatial coordinates `S2015_ELITE_XYZ`.
   - **Stream**: Resultant forces ($Fx, Fy, Fz$) + 52-taxel 3D array data.
3. **Right Tactile Sensor (PaXini PX6AX-GEN3 DP-S2015-Elite)**
   - **Port**: Dedicated USB-UART Board.
   - **Baudrate**: `1000000` bps (1M Baud).
   - **Array Layout**: 52 3D taxels with predefined spatial coordinates `S2015_ELITE_XYZ`.
   - **Stream**: Resultant forces ($Fx, Fy, Fz$) + 52-taxel 3D array data.

---

## 2. Phased Implementation Roadmap

### Phase 1: Environment Setup, Dependencies & Modular Structure
- [x] **Task 1.1: Verify Python Environment & Dependencies**
  - Verify installed packages: `PyQt6`, `pyqtgraph`, `pyserial`, `numpy`, `matplotlib`.
  - Validate Matplotlib QtAgg backend integration (`from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg`).
  - Provide fallback/setup check for OpenGL if 3D acceleration is required.
- [x] **Task 1.2: Codebase Organization & Import Isolation**
  - Establish clean module paths connecting `python_reference/tue_acsi` and `Sensor/paxini_usb_core.py`.
  - Ensure zero circular dependencies between motor controller client and sensor worker threads.
- [x] **Task 1.3: Thread Safety & Concurrency Architecture**
  - Worker threads for sensor acquisition (one dedicated `QThread` or `threading.Thread` per sensor at 1M baud).
  - Worker thread for motor controller communications (prevent serial I/O blocking UI event loop).
  - Non-blocking lock-free / ring-buffer data sharing between background workers and GUI render timer.

---

### Phase 2: Serial Communications & Connection Manager (3 COM Ports)
- [x] **Task 2.1: Unified Serial Port Scanner & Enumeration**
  - Implement dynamic scanning using `serial.tools.list_ports.comports()`.
  - Add auto-refresh button and dropdown menus for:
    - Motor Controller Port (default baud 1000000).
    - Left Sensor Port (fixed/default baud 1000000).
    - Right Sensor Port (fixed/default baud 1000000).
  - Add port collision guard: Disallow selecting the same COM port for multiple roles.
- [x] **Task 2.2: Motor Controller Connection Handler**
  - Instantiate `tue_acsi.AcsiClient` on demand.
  - Implement initial handshake: `sys_ping()`, `sys_info()`, `step_info()`, `servo_ping(1)`.
  - Report firmware version, board status, and step resolution on successful connection.
- [x] **Task 2.3: Dual Tactile Sensor Connection Handlers**
  - Instantiate two instances of `UsbSensorWorker` configured for `baudrate=1000000`.
  - Implement automated device address probing (Address 0x01 fallback to 0x02-0x08).
  - Handle sensor stream initialization and rate calculation ($Hz$).
- [x] **Task 2.4: Master Connect / Disconnect Lifecycle**
  - Implement "Connect All" / "Disconnect All" and per-port individual toggle buttons.
  - Add visual LED status indicators (Red = Disconnected, Yellow = Connecting, Green = Streaming/Ready, Orange = Error).
  - Graceful disconnection routine ensuring serial handles are properly flushed and closed on exit.

---

### Phase 3: Motor Control Module (CoreXY + Z Steppers & STS3215 Gripper)
- [x] **Task 3.1: Stepper Realtime Telemetry Display**
  - Periodic polling loop (10-20 Hz) for `step_position()` and `step_status()`.
  - Digital LCD/numeric readouts for Current X (mm), Current Y (mm), Current Z (mm).
  - Visual driver status indicator (Busy / Idle / Error / Disabled).
- [x] **Task 3.2: Stepper Enable / Disable Toggle**
  - Toggle button calling `step_enable(True)` / `step_enable(False)`.
  - Color-coded button state (Green: Enabled / Drivers Energized; Gray/Red: Disabled / Released).
- [x] **Task 3.3: Velocity & Acceleration Adjusters**
  - DoubleSpinBox / Sliders for tool feed velocity (Range: 1.0 to 100.0 mm/s, default: 25.0 mm/s).
  - DoubleSpinBox / Sliders for tool acceleration (Range: 50.0 to 1000.0 mm/s², default: 300.0 mm/s²).
- [x] **Task 3.4: Press-and-Hold Directional Jogging (X+, X-, Y+, Y-, Z+, Z-)**
  - Implement custom `QPushButton` or signal handlers using `.pressed` and `.released`:
    - **X+**: `pressed` -> `step_jog_x(+velocity, accel=accel)` | `released` -> `step_stop()`
    - **X-**: `pressed` -> `step_jog_x(-velocity, accel=accel)` | `released` -> `step_stop()`
    - **Y+**: `pressed` -> `step_jog_y(+velocity, accel=accel)` | `released` -> `step_stop()`
    - **Y-**: `pressed` -> `step_jog_y(-velocity, accel=accel)` | `released` -> `step_stop()`
    - **Z+**: `pressed` -> `step_jog_z(+velocity, accel=accel)` | `released` -> `step_stop()`
    - **Z-**: `pressed` -> `step_jog_z(-velocity, accel=accel)` | `released` -> `step_stop()`
  - Safety watchdog: Stop motion immediately if mouse leaves button boundary or on connection loss.
  - Emergency Stop (`step_abort()`) dedicated button.
- [x] **Task 3.5: Gripper (STS3215 Servo) Telemetry**
  - Real-time readouts via `servo_feedback(1)` or `servo_get_pos(1)` & `servo_get_volt(1)`:
    - Current Position (steps, 0-4095).
    - Supply Voltage (converted from 0.1V units to Volts, e.g., 12.1 V).
    - Motor Load & Temperature (°C).
- [x] **Task 3.6: Gripper Open / Close Control**
  - Target presets: **Open = 55**, **Close = 420**.
  - Toggle button: "Open Gripper (55)" / "Close Gripper (420)" with current state tracking.
  - Direct position slider or spinbox for fine positioning between 55 and 420.
  - Speed adjustment (steps/s, range 100 to 3000) and acceleration adjustment (range 10 to 200).
  - Automatic torque enablement (`servo_set_torque(True, 1)`) prior to position command.

---

### Phase 4: Tactile Sensor Module (Realtime Plotting, 3D Point Cloud & Tare)
- [x] **Task 4.1: Real-time 2D Scrolling Curves (pyqtgraph)**
  - Dual plotting panels (Left Sensor vs. Right Sensor) or unified comparative plot.
  - Fast ring-buffer data consumer updated via Qt timer at 30-60 FPS.
  - Display metrics: Resultant Force $F_x$, $F_y$, $F_z$, Total Magnitude $|F|$, and Shear $F_{shear}$.
- [x] **Task 4.2: Direction Selection Radio / Checkbox Filter**
  - UI control allowing user to select active plot direction:
    - `(•) Fz (Normal Force)` [Default]
    - `(•) Fx (Lateral Shear Force)`
    - `(•) Fy (Longitudinal Shear Force)`
    - `(•) All Forces (Fx, Fy, Fz)`
  - Instantaneous curve visibility toggle without losing history buffer.
- [x] **Task 4.3: Software Tare / Zeroing System**
  - Dedicated "Tare Left", "Tare Right", and "Tare Both" buttons.
  - Triggers 50-frame baseline accumulation in `UsbSensorWorker.request_tare(50)`.
  - Zeroes both resultant forces and 52 individual taxels.
  - Reset Tare button to restore raw un-tared baseline.
- [x] **Task 4.4: 3D Point Cloud Tactile Visualization (52 Taxels)**
  - Embed 3D canvas (using Matplotlib 3D `Axes3D` on `FigureCanvasQTAgg` or `pyqtgraph.opengl`).
  - Spatial mapping using known geometry `S2015_ELITE_XYZ` (52 points in mm).
  - Render 3D point cloud where each point represents physical taxel position $(x_i, y_i, z_i)$.
  - Dynamic taxel color mapping: Colormap (e.g. `inferno` or `turbo`) modulated by real-time taxel normal force $Fz_i$.
  - Dynamic taxel size scaling proportional to pressure.
- [x] **Task 4.5: 3D Force Magnitude & Direction Vector Arrows (Quiver)**
  - Calculate 3D resultant force vector $[\Delta x, \Delta y, \Delta z] = [Fx, Fy, Fz] \times \text{scale}$.
  - Compute Center of Pressure (CoP) coordinates on the sensor surface.
  - Render 3D arrow originating at the CoP (or sensor origin) pointing in force direction.
  - Arrow length dynamically reflects force magnitude $|F| = \sqrt{Fx^2 + Fy^2 + Fz^2}$.
- [x] **Task 4.6: Real-time Metric Readouts Panel**
  - Numerical table / cards displaying:
    - Normal Force $F_z$ (N), Shear $F_t$ (N), Total $|F|$ (N).
    - Friction utilization ratio $\mu = F_t / F_z$.
    - Surface tilt angle ($\psi$) and shear direction angle ($\theta$).
    - Active contact taxels count and Peak taxel ID + peak force.

---

### Phase 5: Synchronized Data Recording & CSV Logging
- [x] **Task 5.1: Master Record Toggle Button & UI Indicator**
  - PushButton: "Start Recording" -> changes to "Stop Recording" (Red highlight).
  - Live recording status: Elapsed recording time ($hh:mm:ss$), total rows logged, current file path.
- [x] **Task 5.2: Unified Synchronized Data Structure**
  - Background recording thread with non-blocking queue (`queue.SimpleQueue`).
  - Header schema including:
    - Relative time (`t_rel_s`) and UNIX epoch timestamp (`epoch_s`).
    - Stepper coordinates: `Stepper_X_mm`, `Stepper_Y_mm`, `Stepper_Z_mm`.
    - Gripper state: `Gripper_Pos_raw`, `Gripper_Volt_V`.
    - Left Sensor: `L_Fx`, `L_Fy`, `L_Fz`, `L_Fmag`, `L_Fshear`, `L_mu`, `L_CoPx`, `L_CoPy`, `L_ContactTaxels`, plus 52-taxel forces (`L_t1_Fx`..`L_t52_Fz`).
    - Right Sensor: `R_Fx`, `R_Fy`, `R_Fz`, `R_Fmag`, `R_Fshear`, `R_mu`, `R_CoPx`, `R_CoPy`, `R_ContactTaxels`, plus 52-taxel forces (`R_t1_Fx`..`R_t52_Fz`).
- [x] **Task 5.3: High-Throughput Batch Disk Flushing**
  - Background writer flushing batched chunks to disk using `numpy.savetxt` or buffered I/O.
  - Ensures 100+ Hz logging without dropping serial frames or inducing GUI lag.
- [x] **Task 5.4: File Management & Output Directory**
  - Automatic timestamped file naming: `data_record_YYYYMMDD_HHMMSS.csv`.
  - Configurable target directory selector with desktop/workspace defaults.
  - Compatibility with post-processing script `plot_paxini_data.py`.

---

### Phase 6: UI Layout, Ergonomics & Visual Styling
- [x] **Task 6.1: Main Window Layout & Widget Hierarchy**
  - Left Column (Control & Telemetry):
    - Connection Group (3 COM ports, Baudrates, Status LEDs).
    - Stepper Control Group (Positions, Jog Buttons, Speed/Accel, Enable).
    - Gripper Control Group (Voltage, Position, Open/Close, Speed).
    - Recording Group (Start/Stop, Duration, Row count).
  - Right Column (Sensors & Visualization):
    - Plot Direction Filter (Fz / Fx / Fy selector).
    - Real-time 2D Force History Plot (`pyqtgraph`).
    - 3D Tactile Surface & Vector Arrow View (3D Point Cloud).
    - Live Numerical Metrics Grid.
- [x] **Task 6.2: Professional Dark Theme (QSS Styling)**
  - Implement modern dark stylesheet with high-contrast text and clean borders.
  - Differentiated color accents for axes (X=Red, Y=Green, Z=Blue) and sensors (Left=Cyan, Right=Orange).

---

### Phase 7: Edge Cases, Safety & Hardware Verification
- [x] **Task 7.1: Serial Disconnection & Timeout Resilience**
  - Catch `serial.SerialException` and `AcsiTimeout` gracefully without crashing GUI.
  - Auto-disable motion controls if motor serial connection drops.
- [x] **Task 7.2: Motion Safety Watchdogs**
  - Prevent issuing move/jog commands when drivers are disabled (`step_enable == False`).
  - Clamp velocity and acceleration inputs to physical limits.
  - Immediate `step_stop()` triggered upon window minimize, focus loss, or tab switch while jogging.
- [x] **Task 7.3: High-Speed Sensor Synchronization (1M Baud)**
  - Verify serial buffer management at 1,000,000 baud to prevent RX buffer overrun.
  - Monitor frame drop counters and verify steady rate ($\sim 50-100\text{ Hz}$).
- [x] **Task 7.4: Verification Against Offline Analysis Tool**
  - Record sample session with motion and dual contact grasp.
  - Validate generated CSV files with `plot_paxini_data.py` to confirm format compatibility.

---

## 3. Summary of Files & Reference Links
- Stepper & Servo ACSI Client: [`python_reference/tue_acsi/client.py`](file:///C:/Users/nguye/OneDrive/Desktop/Control/python_reference/tue_acsi/client.py)
- Stepper Demo & Jogging API: [`python_reference/examples/stepper_demo.py`](file:///C:/Users/nguye/OneDrive/Desktop/Control/python_reference/examples/stepper_demo.py)
- Gripper Servo Demo & STS3215 Feedback: [`python_reference/examples/servo_demo.py`](file:///C:/Users/nguye/OneDrive/Desktop/Control/python_reference/examples/servo_demo.py)
- Tactile Sensor Core & Ring Buffer: [`Sensor/paxini_usb_core.py`](file:///C:/Users/nguye/OneDrive/Desktop/Control/Sensor/paxini_usb_core.py)
- Tactile Sensor GUI Reference: [`Sensor/paxini_usb_gui.py`](file:///C:/Users/nguye/OneDrive/Desktop/Control/Sensor/paxini_usb_gui.py)
- 3D Coordinates & Offline Visualizer: [`plot_paxini_data.py`](file:///C:/Users/nguye/OneDrive/Desktop/Control/plot_paxini_data.py)

---

## 4. Delivered Implementation

Run the unified application:

```bash
pip install -r requirements.txt
python run_unified_gui.py          # any of these three launch styles works:
python -m unified_gui.app          #   module invocation
python unified_gui/app.py          #   direct file invocation
python -m unified_gui.deps         # pre-flight dependency / backend report
python -m unittest discover -s tests -t .
```

| Module | Responsibility |
|:--|:--|
| `run_unified_gui.py`, `unified_gui/app.py` | Launcher + QApplication/backend setup, dependency gate |
| `unified_gui/_paths.py` | Import isolation for `tue_acsi` and `paxini_usb_core` (Task 1.2) |
| `unified_gui/deps.py` | Environment checks, QtAgg backend + OpenGL fallback report (Task 1.1) |
| `unified_gui/theme.py` | Neutral light QSS theme; colour reserved for status LEDs, stop actions and the pressure map (Task 6.2) |
| `unified_gui/motor.py` | `MotorWorker` thread: handshake, telemetry poll, jog/enable/abort/servo, error resilience (Tasks 2.2, 3.x, 7.1, 7.2) |
| `unified_gui/sensors.py` | `SensorLink`: worker lifecycle, auto-detected device address, tare, health/drop counters (Tasks 2.3, 4.3, 7.3) |
| `unified_gui/haptic.py` | Tactile → 5-valve synthesis (`HapticPayloadSynthesizer`), 20-byte TUE framing at CAN `0x300` (`TueBinaryEncoder`) and the 100 Hz USB CDC streamer (`Stm32HapticStreamer`) - the host bridge of `TODO_SEND_SENSOR_DATA_THROUGH_USB.md` |
| `unified_gui/recording.py` | Master synchronized CSV + offline-compatible per-sensor CSV, batched flushing (Tasks 5.2-5.4) |
| `unified_gui/taxelmap.py` | Live taxel map: 52 taxels at their (x, y) positions, coloured by Fz, CoP marker + shear arrow - the reference reader's view (Task 4.4) |
| `unified_gui/valvemap.py` | Live 5-valve haptic output map: the five pneumatic chambers at their finger positions, coloured by commanded force, with the driving payload byte and frame age |
| `unified_gui/plot3d.py` | 3D taxel point cloud, colormap/size mapping, CoP + force quiver, self-throttled (Tasks 4.4, 4.5) |
| `unified_gui/panels/connection.py` | 4 link roles (motor, both sensors, haptic out), scan/refresh, LED states, collision guard, connect/disconnect (Phase 2) |
| `unified_gui/panels/motor.py` | Stepper telemetry/enable/feed, jog pad (X/Y cross + Z column) + E-stop, gripper open/close/telemetry (Tasks 3.1-3.6) |
| `unified_gui/panels/sensor.py` | 2D scrolling curves, direction radio filter, tare buttons, metrics grid (Tasks 4.1-4.3, 4.6) |
| `unified_gui/panels/recording.py` | Record toggle, elapsed/rows/file display, output-dir quick picks (Tasks 5.1, 5.4) |
| `unified_gui/main_window.py` | Layout, refresh loop, lifecycle wiring, safety watchdog (Tasks 6.1, 7.1, 7.2) |
| `tests/` | Recording-format, worker, sensor-link, haptic synthesis/streaming and offscreen GUI smoke tests (Task 7.4) |

**Hardware note (corrected).** The sensor boards run at **921600** bps, not the
1 000 000 the spec table above claims - that is what `paxini_usb_core.BAUDRATE`
(and therefore the working reference reader) opens them at. `SENSOR_BAUDRATE` now
reads `core.BAUDRATE` directly, so the GUI cannot drift from it again; the value
stays editable per role in the Connection panel. The motor board is left at
1 000 000 bps per the spec (also editable) - `tue_usb.TueUsbClient` itself defaults
to 115200, so check this first if the stepper link will not come up.

**Port topology.** Three COM ports in total: **one** for the STM32 master board
and **one per tactile sensor**. The STM32 port carries both motor control and
the 100 Hz haptic output frames — the haptic streamer borrows
`MotorWorker.write_frame` (which writes under the same lock as the motor client)
instead of opening a port, so the two protocols are multiplexed frame-by-frame
on one cable. The "Haptic output" panel follows the STM32 link automatically.
Start/stop is per-session; the streamer only opens its own port when no writer
is supplied (bench use).

**Device address.** The sensor bus address is auto-detected on connect: the
calibration request is sent to every candidate address (`0x01`-`0x08`) and the first
one that answers is used, with the sweep repeated so a still-booting board is still
found (`paxini_usb_core.probe_device_address`). The resolved address is shown in the
Connection panel (`ID 0x03 | 47.8 Hz | drops 6 (0.1%)`) and in the log. An explicit
address can still be pinned via `SensorLink(device_addr=0x05)`; it is probed first but
the remaining range is still swept. If nothing answers, the link reports
`no sensor answered on addresses 01-08` and the error LED turns orange instead of
silently streaming nothing.

**Look and feel.** The UI is deliberately monochrome on a light background:
light greys with one text colour. Colour is kept only where it carries
information - the four connection status LEDs, the stop/destructive actions
(E-stop, Stop Recording), and the 3D pressure map (greyscale by default, so idle
taxels sit light and pressure darkens them; hue maps are selectable in the
toolbar). Sensor curves are separated by lightness (Left dark, Right mid grey)
and pen style (Fz solid, Fx dashed, Fy dotted) rather than by hue. The whole
palette lives in `unified_gui/theme.py`, so switching it back to dark is a
one-file change.

**Jog pad.** X and Y share a four-button cross - the X pair stacked vertically
(X- above X+) and the Y pair side by side horizontally (Y+ left of Y-), with the
centre of the cross deliberately empty. Z is a separate two-button column (Z-
above Z+), divided from the cross by a rule, with the E-stop spanning the full
height to its right. `tests/test_gui_smoke.py::test_jog_pad_layout` pins these
relative positions.

**Tactile views.** The "Tactile surface & haptic output" panel is tabbed. The
default tab is the live taxel map (`unified_gui/taxelmap.py`) - the same
pyqtgraph view as the reference reader, so it stays live at the full 30 FPS. The
second tab is the Matplotlib 3D canvas (allowed by Task 4.4); it is CPU-rendered,
so its redraw rate is user-adjustable (default 4 Hz), it can be paused, and it is
only redrawn while its tab is actually visible. Installing `PyOpenGL` enables the
optional GPU path reported by `unified_gui.deps`. The third tab is the 5-valve
haptic output map (`unified_gui/valvemap.py`).

**Haptic output (STM32 bridge).** `unified_gui/haptic.py` implements the host
side of `TODO_SEND_SENSOR_DATA_THROUGH_USB.md`: the dual 52-taxel jaws are
compressed into one 8-byte payload mapping onto the slave thimble's five valves
(A/B/C = shear steered tip→mid→base, D/E = bilateral clamp force), wrapped in the
same 20-byte TUE binary frame used everywhere else on the link - only the CAN ID
differs (`0x300`, the reserved user range, which is why the streamer writes raw
frames rather than going through `TueUsbClient.transact`). It carries a fourth
connection role ("STM32 Haptic Out", 1 Mbaud) and streams at 100 Hz; on Windows
the process timer resolution is raised to 1 ms for the duration of the stream so
`time.sleep` can actually hold a 10 ms period. The "Haptic valves (5)" tab draws
the five chambers at their finger positions, coloured by the commanded force,
each annotated with its payload byte and the age of the frame that drove it -
and keeps previewing the mapping (marked "preview") even before the output link
is connected.
