# TUE DC Motor — DRV8872 Control Instruction Manual

> **Platform**: STM32F407VET6 (`MM_Control`)  
> **Firmware**: [`Core/Inc/tue_dc.h`](../Core/Inc/tue_dc.h), [`Core/Src/tue_dc.c`](../Core/Src/tue_dc.c)  
> **Protocol**: [`Core/Inc/tue_usb_binary.h`](../Core/Inc/tue_usb_binary.h) (`0x600 – 0x607`)  
> **Host Client**: [`python/tue_usb`](../python/tue_usb) (`TueUsbClient`, `DcMode`, `DcInfo`, `DcChannelStatus`, `DcFaults`)  
> **Timer Base**: TIM4 (Motors 1 & 2) and TIM3 (Motors 3 & 4) at 84 MHz  
> **Related**: [`docs/USB_BINARY_PROTOCOL.md`](USB_BINARY_PROTOCOL.md), [`docs/STEPPER_INSTRUCTION.md`](STEPPER_INSTRUCTION.md), [`docs/STS3215_INSTRUCTION.md`](STS3215_INSTRUCTION.md)

---

## 1. Overview

The `tue_dc` driver controls **four independent brushed DC motor channels** powered by Texas Instruments **DRV8872** H-bridge drivers.

Key capabilities:
- **Bidirectional Speed Control**: Full 4-quadrant drive via PWM from `-100.0%` to `+100.0%` with `0.1%` resolution (per-mille range `-1000` to `+1000`).
- **Ultrasonic 20 kHz PWM**: High-frequency carrier eliminates audible motor whine and current ripple.
- **Active Dynamic Braking**: Low-side slow-decay dynamic brake (both low-side MOSFETs conduct) for immediate rotor deceleration.
- **Hi-Z Coast / Sleep**: Low-power standby and freewheeling coast.
- **Hardware Fault Telemetry & EXTI Interrupts**: Sub-microsecond instantaneous shut-off via EXTI falling-edge interrupts on open-drain active-low `nFAULT` feedback pins (`DC_F1` to `DC_F4`).
- **Timer Update Interrupt Control Loop**: Hardware timer update interrupts (`TIM3_IRQn`, `TIM4_IRQn`) providing a deterministic 1 kHz control tick for smooth velocity acceleration ramping and continuous watchdog supervision.
- **Auto-Protection**: Automatic shut-off to coast mode upon active fault detection to protect coils and drivers.
- **Synchronized Multi-Motor Drive**: Single USB CAN frame sets all 4 channels simultaneously for coordinated multi-actuator robotics.

---

## 2. Hardware Architecture & Pin Assignments

```
                STM32F407VET6 (MM_Control)
            ┌────────────────────────────────┐
            │                                │
            │   PD12 (TIM4_CH1) ───────────► │ IN1 ──┐
            │   PD13 (TIM4_CH2) ───────────► │ IN2   ├─► Motor 1
            │   PD8  (DC_F1, Input+PU) ◄──── │ nFLT ─┘   (DRV8872 #1)
            │                                │
            │   PD14 (TIM4_CH3) ───────────► │ IN1 ──┐
            │   PD15 (TIM4_CH4) ───────────► │ IN2   ├─► Motor 2
            │   PD9  (DC_F2, Input+PU) ◄──── │ nFLT ─┘   (DRV8872 #2)
            │                                │
            │   PC6  (TIM3_CH1) ───────────► │ IN1 ──┐
            │   PC7  (TIM3_CH2) ───────────► │ IN2   ├─► Motor 3
            │   PD10 (DC_F3, Input+PU) ◄──── │ nFLT ─┘   (DRV8872 #3)
            │                                │
            │   PC8  (TIM3_CH3) ───────────► │ IN1 ──┐
            │   PC9  (TIM3_CH4) ───────────► │ IN2   ├─► Motor 4
            │   PD11 (DC_F4, Input+PU) ◄──── │ nFLT ─┘   (DRV8872 #4)
            │                                │
            └────────────────────────────────┘
```

### 2.1 Complete Pinout Table

| Channel | Label | Input 1 (IN1) | Input 2 (IN2) | Timer & Channels | Fault Feedback (nFAULT) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **Motor 1** | M1 | `PD12` (`DC_11`) | `PD13` (`DC_12`) | `TIM4_CH1` / `TIM4_CH2` | `PD8` (`DC_F1`) |
| **Motor 2** | M2 | `PD14` (`DC_21`) | `PD15` (`DC_22`) | `TIM4_CH3` / `TIM4_CH4` | `PD9` (`DC_F2`) |
| **Motor 3** | M3 | `PC6`  (`DC_31`) | `PC7`  (`DC_32`) | `TIM3_CH1` / `TIM3_CH2` | `PD10` (`DC_F3`) |
| **Motor 4** | M4 | `PC8`  (`DC_41`) | `PC9`  (`DC_42`) | `TIM3_CH3` / `TIM3_CH4` | `PD11` (`DC_F4`) |

---

## 3. DRV8872 H-Bridge Driver Theory of Operation

### 3.1 Control Logic Truth Table

The DRV8872 H-bridge truth table governs motor states:

| IN1 | IN2 | OUT1 | OUT2 | Operating Mode | Description |
|:---:|:---:|:---:|:---:|:---:|:---|
| `0` | `0` | **Hi-Z** | **Hi-Z** | `TUE_DC_MODE_COAST` | Coast / Sleep / Fast-decay freewheel |
| `1` | `0` | **H** | **L** | `TUE_DC_MODE_FORWARD` | Forward drive (current flows OUT1 $\rightarrow$ OUT2) |
| `0` | `1` | **L** | **H** | `TUE_DC_MODE_REVERSE` | Reverse drive (current flows OUT2 $\rightarrow$ OUT1) |
| `1` | `1` | **L** | **L** | `TUE_DC_MODE_BRAKE` | Low-side slow-decay active dynamic brake |

### 3.2 Bidirectional PWM Speed Regulation

Speed commands are scaled from `-1000` to `+1000` (`int16_t`):

1. **Forward Drive (`speed > 0`)**:
   - `IN1`: Generates PWM waveform proportional to $|speed|$.
   - `IN2`: Held solidly at logic `0` (`CCR = 0`).
   - During PWM ON time, output is driven Forward (`OUT1=H, OUT2=L`). During PWM OFF time, outputs are `Hi-Z` (fast decay).
2. **Reverse Drive (`speed < 0`)**:
   - `IN1`: Held solidly at logic `0` (`CCR = 0`).
   - `IN2`: Generates PWM waveform proportional to $|speed|$.
   - During PWM ON time, output is driven Reverse (`OUT1=L, OUT2=H`).
3. **Zero Speed (`speed == 0`)**:
   - Both `IN1 = 0` and `IN2 = 0` (`CCR1 = 0, CCR2 = 0`).
   - H-bridge enters low-power idle coast mode.
4. **Active Dynamic Brake**:
   - Both `IN1 = 1` and `IN2 = 1` (`CCR1 = ARR + 1, CCR2 = ARR + 1`).
   - Both low-side MOSFETs conduct, shorting back-EMF through ground and rapidly arresting rotation.

### 3.3 Fault Feedback (`nFAULT`)

- `nFAULT` is an **active-low, open-drain** output on the DRV8872.
- The STM32 GPIO inputs `PD8..PD11` are initialized with internal pull-ups (`GPIO_PULLUP`).
- Under normal operating conditions, `nFAULT` is pulled **HIGH** (`3.3 V`).
- If an internal protection event occurs, `nFAULT` is driven **LOW** (`0 V`):
  - **OCP (Overcurrent Protection)**: Load current exceeds $I_{\text{TRIP}}$ limit.
  - **TSD (Thermal Shutdown)**: Junction temperature exceeds $160^\circ\text{C}$.
  - **UVLO (Undervoltage Lockout)**: Motor voltage $V_{VM}$ drops below $6.5\text{ V}$.
- When `auto_fault_protection` is enabled (default), detecting `nFAULT == 0` immediately commands the channel to `COAST` (`0%` duty) and latches the diagnostic flag.

---

## 4. Timer Clock & PWM Mathematics

The STM32F407 runs at $168\text{ MHz}$ core clock with APB1 prescaler set to $/4$:
- $\text{PCLK1} = 168 / 4 = 42\text{ MHz}$.
- Since APB1 prescaler $> 1$, timer peripheral clock multiplier is $\times 2$:
  $$\text{Timer Clock} = 42\text{ MHz} \times 2 = 84\text{ MHz}$$

### 4.1 Frequency & Period Calculation

Targeting an ultrasonic carrier frequency $f_{\text{PWM}} = 20\text{ kHz}$:
$$\text{Period Count} = \frac{84\text{ MHz}}{(\text{PSC} + 1) \times f_{\text{PWM}}} = \frac{84,000,000}{(0 + 1) \times 20,000} = 4200$$

Therefore:
- **Prescaler (`PSC`)**: `0`
- **Auto-Reload Register (`ARR`)**: $4200 - 1 = \mathbf{4199}$
- **PWM Frequency**: $\frac{84,000,000}{4200} = \mathbf{20,000\text{ Hz}}$ (exact 20.0 kHz, silent)
- **Resolution**: 4200 discrete counter steps (higher than 12-bit timer resolution).

### 4.2 Duty Cycle Scaling

For speed $S \in [0, 1000]$:
$$\text{Pulse} = \left\lfloor \frac{S \times (\text{ARR} + 1)}{1000} \right\rfloor$$
- When $S = 0$: $\text{Pulse} = 0$ (solid LOW, 0.0% duty).
- When $S = 500$: $\text{Pulse} = 2100$ (50.0% square wave).
- When $S = 1000$: $\text{Pulse} = 4200$ (solid HIGH, 100.0% duty).

---

## 5. Firmware C API Reference

Header: [`Core/Inc/tue_dc.h`](../Core/Inc/tue_dc.h)  
Source: [`Core/Src/tue_dc.c`](../Core/Src/tue_dc.c)

### 5.1 Enums & Types

```c
typedef enum {
    TUE_DC_OK             = 0,
    TUE_DC_ERR_PARAM      = 1,
    TUE_DC_ERR_FAULT      = 2,
    TUE_DC_ERR_HAL        = 3,
    TUE_DC_ERR_BUSY       = 4
} tue_dc_status_t;

typedef enum {
    TUE_DC_MODE_COAST     = 0,  /* Hi-Z coast / standby (IN1=0, IN2=0) */
    TUE_DC_MODE_FORWARD   = 1,  /* Forward rotation (IN1=PWM, IN2=0) */
    TUE_DC_MODE_REVERSE   = 2,  /* Reverse rotation (IN1=0, IN2=PWM) */
    TUE_DC_MODE_BRAKE     = 3   /* Active dynamic brake (IN1=1, IN2=1) */
} tue_dc_mode_t;

typedef struct {
    uint8_t       motor_id;       /* 1..4 */
    tue_dc_mode_t mode;           /* COAST / FORWARD / REVERSE / BRAKE */
    int16_t       speed;          /* Commanded speed (-1000..+1000) */
    uint16_t      duty;           /* Absolute duty (0..1000) */
    bool          fault_active;   /* Current instantaneous nFAULT pin */
    bool          fault_latched;  /* Sticky fault event latch */
    uint32_t      fault_count;    /* Number of fault transitions */
} tue_dc_channel_status_t;
```

### 5.2 Functions

```c
/* Initialization & Lifecycle */
tue_dc_status_t tue_dc_init(void);
bool            tue_dc_is_initialized(void);

/* Speed Control */
tue_dc_status_t tue_dc_set_speed(uint8_t motor_id, int16_t speed);
tue_dc_status_t tue_dc_set_speed_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/* Stopping & Braking */
tue_dc_status_t tue_dc_brake(uint8_t motor_id); /* 1..4 or 0xFF for all */
tue_dc_status_t tue_dc_coast(uint8_t motor_id); /* 1..4 or 0xFF for all */
tue_dc_status_t tue_dc_stop(uint8_t motor_id);

/* Telemetry & Diagnostics */
tue_dc_status_t tue_dc_get_channel_status(uint8_t motor_id, tue_dc_channel_status_t *out_status);
tue_dc_status_t tue_dc_get_info(tue_dc_info_t *out_info);
bool            tue_dc_get_fault(uint8_t motor_id);
uint8_t         tue_dc_get_fault_mask(void);
void            tue_dc_clear_faults(void);

/* Frequency & Safety */
tue_dc_status_t tue_dc_set_pwm_frequency(uint32_t freq_hz);
uint32_t        tue_dc_get_pwm_frequency(void);
void            tue_dc_set_auto_fault_protection(bool enable);
bool            tue_dc_get_auto_fault_protection(void);

/* Interrupt Handlers & Velocity Ramping */
void            tue_dc_on_fault_exti(uint16_t GPIO_Pin);
void            tue_dc_on_timer_elapsed(TIM_HandleTypeDef *htim);
void            tue_dc_set_ramp_rate(uint16_t rate_per_s);
uint16_t        tue_dc_get_ramp_rate(void);
```

---

## 6. Binary USB Protocol Integration

The DC motor subsystem commands reside in the CAN ID range **`0x600 – 0x607`**:

| CAN ID | Name | Direction | Req DLC | Request Payload | Resp DLC | Response Payload |
|:---:|:---|:---:|:---:|:---|:---:|:---|
| `0x600` | `DC_GET_INFO` | Host $\rightarrow$ MCU | 0 | — | 8 | `motor_count (u8), freq_khz (u8), max_duty (u16 LE), timer_arr (u16 LE), driver_type (u16 LE)` |
| `0x601` | `DC_SET_SPEED` | Host $\rightarrow$ MCU | 3 | `motor_id (u8: 1..4), speed (i16 LE: -1000..+1000)` | 4 | `motor_id (u8), current_speed (i16 LE), fault_active (u8)` |
| `0x602` | `DC_GET_STATUS` | Host $\rightarrow$ MCU | 1 | `motor_id (u8: 1..4)` | 6 | `motor_id (u8), mode (u8: 0..3), speed (i16 LE), fault_active (u8), fault_latched (u8)` |
| `0x603` | `DC_SET_SPEED_ALL` | Host $\rightarrow$ MCU | 8 | `m1 (i16 LE), m2 (i16 LE), m3 (i16 LE), m4 (i16 LE)` | 5 | `fault_mask (u8), m1_mode (u8), m2_mode (u8), m3_mode (u8), m4_mode (u8)` |
| `0x604` | `DC_BRAKE` | Host $\rightarrow$ MCU | 1 | `motor_id (u8: 1..4, 0xFF=all)` | 2 | `motor_id (u8), mode (u8: 3=BRAKE)` |
| `0x605` | `DC_COAST` | Host $\rightarrow$ MCU | 1 | `motor_id (u8: 1..4, 0xFF=all)` | 2 | `motor_id (u8), mode (u8: 0=COAST)` |
| `0x606` | `DC_GET_FAULTS` | Host $\rightarrow$ MCU | 0 | — | 5 | `fault_mask (u8), m1_fault (u8), m2_fault (u8), m3_fault (u8), m4_fault (u8)` |
| `0x607` | `DC_CONFIG_FREQ` | Host $\rightarrow$ MCU | 2 | `freq_khz (u16 LE: 1..50)` | 4 | `actual_freq_hz (u32 LE)` |

---

## 7. Python Host API Reference & Examples

### 7.1 Single Motor Speed Control

```python
from tue_usb import TueUsbClient, DcMode

with TueUsbClient("COM7") as dev:
    # Read subsystem capabilities
    info = dev.dc_get_info()
    print(f"DC Controller: {info.motor_count} channels, {info.pwm_freq_khz} kHz, Driver={info.driver_name}")

    # Drive Motor 1 Forward at 75% duty
    m_id, speed, fault = dev.dc_set_speed(1, 750)
    print(f"Motor {m_id} running at {speed / 10}% duty (Fault={fault})")

    # Read live telemetry
    st = dev.dc_get_status(1)
    print(f"Mode={st.mode.name}, Speed={st.speed}, FaultActive={st.fault_active}")

    # Active dynamic brake
    dev.dc_brake(1)
```

### 7.2 Synchronized 4-Channel Control

```python
with TueUsbClient("COM7") as dev:
    # Set Motor 1=100%, Motor 2=-50%, Motor 3=80%, Motor 4=Coast in a single frame
    mask, m1, m2, m3, m4 = dev.dc_set_speed_all(1000, -500, 800, 0)
    print(f"Fault mask: 0x{mask:02X}, Modes: M1={m1.name}, M2={m2.name}, M3={m3.name}, M4={m4.name}")

    # Emergency coast all channels
    dev.dc_coast(0xFF)
```

### 7.3 Fault Monitoring

```python
with TueUsbClient("COM7") as dev:
    faults = dev.dc_get_faults()
    if faults.has_any_fault():
        print(f"ALERT! Fault detected on channels: "
              f"M1={faults.m1_fault}, M2={faults.m2_fault}, "
              f"M3={faults.m3_fault}, M4={faults.m4_fault}")
    else:
        print("All 4 DRV8872 drivers operating nominally (no faults).")
```

---

## 8. Verification & Test Suite

The implementation is verified across firmware and host software:
- **Firmware Compilation**: Builds cleanly via CMake / Ninja with Arm GNU Toolchain 15.2 (`MM_control.elf` 0 warnings).
- **Python Unit Tests**: Integrated in `python/tests/test_protocol.py`:
  - `test_10_dc_motor_protocol`: Tests binary serialization, frame boundaries, and endianness for all DC CAN IDs (`0x600 – 0x607`).
  - `test_11_mock_client_dc_operations`: Validates `TueUsbClient` methods against simulated mock state machine.
  - Non-collision assertion checks verifying no DC ID overlaps with the reserved `0x3...` range.

---

## 9. Standalone Hardware Test (Pre-FreeRTOS Debug Mode)

To isolate board-level hardware wiring issues, DRV8872 IC mounting faults, or power supply limits from RTOS task interactions, a standalone test function is provided:

- Function: `tue_dc_test_low_speed_loop()` in [`Core/Src/tue_dc.c`](../Core/Src/tue_dc.c)
- Configuration macro in [`Core/Src/main.c`](../Core/Src/main.c): `#define DC_DEBUG_STANDALONE 1`

### 9.1 Test Cycle Behavior

```
               ◄──────────────────────── Continual Cycle ────────────────────────►
┌───────────────────────┬───────────────────┬───────────────────────┬───────────────────┐
│   FORWARD (+25% PWM)  │   DYNAMIC BRAKE   │   REVERSE (-25% PWM)  │   COAST (0% PWM)  │
│      3.0 seconds      │    1.0 second     │      3.0 seconds      │    1.0 second     │
│   LED blinks 100 ms   │   Fault check     │   LED blinks 100 ms   │   Fault check     │
└───────────────────────┴───────────────────┴───────────────────────┴───────────────────┘
```

1. **Forward Drive**: All 4 channels driven at $+25.0\%$ duty (`speed = +250`).
2. **Active Brake**: All 4 channels actively dynamic-braked (`IN1=1, IN2=1`).
3. **Reverse Drive**: All 4 channels driven at $-25.0\%$ duty (`speed = -250`).
4. **Coast**: All 4 channels freewheel in Hi-Z standby.
5. **Continuous Fault Monitoring**:
   - `DC_F1` to `DC_F4` (`PD8`..`PD11`) are sampled continuously.
   - If any `nFAULT` pin pulls LOW, the test immediately logs the faulted channel in `g_dc_test_fault_mask` and triggers a rapid visual alert blink (50 ms bursts) on `LED_MCU` (`PE5`).

### 9.2 Live Debugger Variables

When connecting with GDB or STM32CubeIDE Live Watch:
- `g_dc_test_step`: `1` = Forward, `2` = Brake, `3` = Reverse, `4` = Coast.
- `g_dc_test_fault_mask`: Bitmask of active faults (`bit0` = Motor 1, `bit1` = Motor 2, `bit2` = Motor 3, `bit3` = Motor 4).
- `g_dc_test_cycle_count`: Monotonically increasing loop counter.

### 9.3 Toggling Standalone Test vs FreeRTOS

In [`Core/Src/main.c`](../Core/Src/main.c):
```c
/* Set to 1 to run standalone DC motor test loop (FreeRTOS scheduler bypassed).
 * Set to 0 to resume normal operation with FreeRTOS scheduler & USB binary protocol. */
#define DC_DEBUG_STANDALONE 1
```

---

## 10. Interrupt Architecture (EXTI & Timer Update)

To achieve hard-real-time safety and smooth, jitter-free speed ramping without burdening the FreeRTOS scheduler, the DC motor subsystem integrates two complementary hardware interrupt pathways:

```
                            ┌──────────────────────────────────────────────┐
                            │ DRV8872 Fault Event (OCP / TSD / UVLO)       │
                            │ nFAULT Pin Falls LOW (PD8..PD11)             │
                            └──────────────────────┬───────────────────────┘
                                                   │
                                                   ▼
                                 ┌───────────────────────────────────┐
                                 │ EXTI9_5_IRQn / EXTI15_10_IRQn     │
                                 │ Priority 5, Subpriority 0         │
                                 └─────────────────┬─────────────────┘
                                                   │
                                                   ▼
                                 ┌───────────────────────────────────┐
                                 │ tue_dc_on_fault_exti(pin)         │
                                 │ - Instantly sets CCR1=0, CCR2=0   │
                                 │ - Halts PWM in < 1 µs             │
                                 │ - Latches fault telemetry flag    │
                                 └───────────────────────────────────┘

                            ┌──────────────────────────────────────────────┐
                            │ TIM3 & TIM4 PWM Counter Overflow (20 kHz)    │
                            │ Hardware Auto-Reload (ARR) Event             │
                            └──────────────────────┬───────────────────────┘
                                                   │
                                                   ▼
                                 ┌───────────────────────────────────┐
                                 │ TIM3_IRQn / TIM4_IRQn             │
                                 │ Priority 5, Subpriority 0         │
                                 └─────────────────┬─────────────────┘
                                                   │
                                                   ▼
                                 ┌───────────────────────────────────┐
                                 │ tue_dc_on_timer_elapsed(htim)     │
                                 │ Prescaler /20 -> 1 kHz Tick (1 ms)│
                                 │ TIM4: Motors 1 & 2                │
                                 │ TIM3: Motors 3 & 4                │
                                 └─────────────────┬─────────────────┘
                                                   │
                                                   ▼
                                 ┌───────────────────────────────────┐
                                 │ dc_service_channel_tick(idx)      │
                                 │ - Continuous fault verification   │
                                 │ - Linear velocity slew ramping    │
                                 │   towards s_target_speed[idx]     │
                                 └───────────────────────────────────┘
```

### 10.1 EXTI Falling-Edge Hardware Fault Protection

- **Pins**: `PD8` (`DC_F1`), `PD9` (`DC_F2`), `PD10` (`DC_F3`), `PD11` (`DC_F4`).
- **EXTI Vectors**:
  - `PD8` & `PD9`: `EXTI9_5_IRQn`
  - `PD10` & `PD11`: `EXTI15_10_IRQn`
- **Interrupt Mode**: `GPIO_MODE_IT_FALLING` with `GPIO_PULLUP`.
- **NVIC Priority**: Preemption priority `5`, subpriority `0` (compatible with FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`).
- **Sub-Microsecond Response**:
  As soon as the DRV8872 pulls `nFAULT` low, the EXTI ISR invokes:
  ```c
  void tue_dc_on_fault_exti(uint16_t GPIO_Pin);
  ```
  This immediately executes:
  ```c
  dc_apply_hardware(idx, 0U, 0U);
  ```
  Zeroing both timer capture/compare registers (`CCR1 = 0, CCR2 = 0`) directly shuts off gate drive to the external bridge in **under 1 microsecond**, preventing thermal destruction or board-level trace overcurrent damage.

### 10.2 TIM3 and TIM4 Timer Update Interrupts & 1 kHz Ramping Loop

- **Source**: Timer update interrupt on overflow (`TIM_IT_UPDATE`) enabled on `&htim3` and `&htim4`.
- **Carrier vs Control Loop**:
  - Timer ARR rolls over at $20\text{ kHz}$ ($50\,\mu\text{s}$ period).
  - Inside `tue_dc_on_timer_elapsed(htim)`, a software prescaler divides by $20$ ($20{,}000 / 20 = 1{,}000\text{ Hz}$), executing channel servicing at exactly $1\text{ ms}$ intervals.
  - Channel division:
    - **TIM4**: Services **Motor 1** (idx 0) and **Motor 2** (idx 1).
    - **TIM3**: Services **Motor 3** (idx 2) and **Motor 4** (idx 3).
- **Smooth Acceleration / Slew Rate Control**:
  - Set via `tue_dc_set_ramp_rate(rate_per_s)`.
  - When `rate_per_s > 0`, commanded speeds via `tue_dc_set_speed()` or USB `0x601 / 0x603` update `s_target_speed[idx]`.
  - Each 1 ms tick increments or decrements current speed towards target by:
    $$\Delta S = \max\left(1, \left\lfloor \frac{\text{rate\_per\_s}}{1000} \right\rfloor\right)$$
  - This eliminates mechanical shock, reduces gear wear, and prevents power rail brownouts from high inrush currents.
  - If `rate_per_s == 0`, speed transitions remain instantaneous for high-bandwidth robotic haptic feedback.

