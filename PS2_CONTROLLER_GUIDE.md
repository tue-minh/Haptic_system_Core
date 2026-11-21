# PS2 Wireless RX Board ↔ `STM32_FIRMWARE` (`SPI3`) — Connection & Firmware Guide

Complete hardware wiring, `MM_control.ioc` (STM32CubeMX) modification checklist, and STM32 HAL + FreeRTOS firmware guide for receiving commands from a PlayStation 2 handheld controller via a **PS2 Wireless RX Receiver Board** on **`SPI3`** of **`STM32_FIRMWARE`** (`STM32F407VETx`, `168 MHz`, FreeRTOS).

> **Protocol Source:** [Curious Inventor — Interfacing a PS2 (PlayStation 2) Controller](https://store.curiousinventor.com/guides/PS2/)

---

## 1. System Architecture (`SPI3`)

In this setup, the **STM32F407 replaces the PlayStation console** and acts as the **SPI Master** on **`SPI3`** (`PC10`, `PC11`, `PC12` + one `SPI3_CSx` pin on `GPIOD`). The **PS2 Wireless RX board** acts as the **SPI Slave**.

```
+-----------------------+       2.4 GHz RF       +---------------------+       SPI3 (3.3 V)        +----------------------+
|  PS2 Handheld         | <===================>  |  PS2 Wireless RX    | <=====================>   |  STM32F407VETx       |
|  Controller (Battery) |                        |  Board (SPI Slave)  |  PC10/11/12 + PD4 (CS1)   |  (STM32_FIRMWARE)    |
+-----------------------+                        +---------------------+                           |  SPI3 Master         |
                                                                                                   +----------------------+
```

> **Note on Existing `SPI3` Usage in `STM32_FIRMWARE`:**
> `MM_control.ioc` already enables `SPI3` with three software chip-select pins (`PD4` = `SPI3_CS1`, `PD3` = `SPI3_CS2`, `PD2` = `SPI3_CS3`), and `SPI3` is now **reserved exclusively for the PS2 receiver**.
> - `Core/Src/tue_spi_adc.c` only maps the F103 ADC slaves onto `SPI1` (logical indices `0..2`, CS1 `PB8`, CS2 `PB9`, CS3 `PE0`). Default active bus is `SPI1` (`bus_id = 1`).
> - `tue_spi_adc_configure()` now **rejects any `bus_id` other than `1`** with `BAD_ARG`, so CAN `0x502` cannot steal `SPI3` or toggle `PD4`. Both GUIs only expose `SPI1`.
> - No FreeRTOS mutex or peripheral re-init is needed any more: the two peripherals are independent.

---

## 2. Hardware Connection (PS2 RX Board → STM32F407 `SPI3`)

### 2.1 PS2 Connector Pinout (Looking Into Plug / RX Header, Pins 1 → 9)

```
   _________________________
  |  1 2 3 | 4 5 6 | 7 8 9  |
  |  o o o | o o o | o o o  |
   \_______|_______|_______/
```

Only **6 wires** are required for a Wireless RX board (`DAT`, `CMD`, `GND`, `3.3V`, `ATT`, `CLK`). Pin 3 (`V_MOTORS`) is **not needed** for a wireless RX board because the handheld controller's batteries power its vibration motors.

| PS2 Pin | Common Wire Color | PS2 Signal | Direction | STM32F407 Pin (`SPI3`) | Notes |
|---:|---|---|---|---|---|
| **1** | Brown | **DAT** (Data) | RX Board → STM32 | **`PC11`** (`SPI3_MISO`) | **Open-collector.** Add an external **4.7 kΩ – 10 kΩ pull-up resistor** to `3.3V` (STM32 internal ~40 kΩ pull-up is often too weak for clean rising edges). |
| **2** | Orange | **CMD** (Command) | STM32 → RX Board | **`PC12`** (`SPI3_MOSI`) | Push-pull output from STM32. |
| **3** | Grey / Violet | **V_MOTORS** | — | *Not connected* | Only for **wired** controllers (6–9 V, ≥500 mA). Leave unconnected for wireless RX boards. |
| **4** | Black | **GND** | Common GND | **`GND`** | Common ground. |
| **5** | Red | **VCC** | Power → RX Board | **`3.3V`** | **Must be 3.3 V.** Many wireless PS2 RX boards fail at 5 V (and STM32F407 logic is 3.3 V). |
| **6** | Yellow | **ATT** (CS / Select) | STM32 → RX Board | **`PD4`** (`SPI3_CS1`) | Active-LOW chip select (`PD3` `SPI3_CS2` or `PD2` `SPI3_CS3` also work if preferred). Must go LOW before each packet and HIGH after. |
| **7** | Blue | **CLK** (Clock) | STM32 → RX Board | **`PC10`** (`SPI3_SCK`) | Idle HIGH, `164.06 kHz` (`APB1 42 MHz / 256`). |
| **8** | White | **N/C** | — | *Not connected* | Unused / unknown. |
| **9** | Green | **ACK** | RX Board → STM32 | *Not connected* (optional) | Open-collector. Not required when using a fixed 20 µs inter-byte delay in firmware. |

### 2.2 Wiring Diagram

```
                        3.3V
                         |
                     [4.7k - 10k]
                         |
  STM32F407 (SPI3)       |         PS2 Wireless RX Board
 +------------------+    |        +----------------------+
 | PC11 (SPI3_MISO) |<---+--------| 1: DAT  (Brown)      |
 | PC12 (SPI3_MOSI) |------------>| 2: CMD  (Orange)     |
 |                  |             | 3: N/C  (V_MOTORS)   |
 | GND              |-------------| 4: GND  (Black)      |
 | 3.3V             |------------>| 5: VCC  (Red)        |
 | PD4  (SPI3_CS1)  |------------>| 6: ATT  (Yellow)     |
 | PC10 (SPI3_SCK)  |------------>| 7: CLK  (Blue)       |
 |                  |             | 8: N/C  (White)      |
 |                  |             | 9: ACK  (Green, opt) |
 +------------------+             +----------------------+
```

> **Note on SPI Master vs. Slave:** Because the STM32 talks directly to the PS2 RX board (no PlayStation console attached), `SPI3` **must be SPI Master**. (Only configure an MCU as SPI Slave if passively sniffing a live, powered-on PlayStation console's bus.)

---

## 3. What to Change in `MM_control.ioc` (STM32CubeMX) for `SPI3`

In `STM32_FIRMWARE/MM_control.ioc`, **`SPI3` is already configured as Master on `PC10`/`PC11`/`PC12`** with `PD4` (`SPI3_CS1`), `PD3` (`SPI3_CS2`), and `PD2` (`SPI3_CS3`) already set as `GPIO_Output` with Initial Pin State = `GPIO_PIN_SET` (HIGH).

However, **`SPI3` currently uses settings for the STM32F103 ADC slave that are incompatible with PS2 (`2.625 MHz`, `CPOL=LOW`, `CPHA=1EDGE`, `MSB First`)**. You **must change 4 parameters** in `SPI3` before regenerating code:

### 3.1 CubeMX GUI Changes (`Connectivity → SPI3 → Parameter Settings`)

`STM32_FIRMWARE` runs `SYSCLK = 168 MHz` and **`APB1 = 42 MHz`**.

| CubeMX Setting | Current Value in `MM_control.ioc` | **Change To (For PS2)** | Why |
|---|---|---|---|
| **Mode** | `Full-Duplex Master` | *Keep `Full-Duplex Master`* | STM32 drives `CLK` and `CMD`, reads `DAT` |
| **Hardware NSS Signal** | `Disable` | *Keep `Disable`* | `PD4` (`SPI3_CS1`) is software-controlled |
| **Frame Format** | `Motorola` | *Keep `Motorola`* | Standard SPI |
| **Data Size** | `8 Bits` | *Keep `8 Bits`* | 1 byte per transfer |
| **First Bit** | `MSB First` | **`LSB First`** | **Critical:** PS2 transmits bit 0 first |
| **Prescaler (for Baud Rate)** | `16` (`2.625 MBits/s`) | **`256`** (`164.0625 KBits/s`) *(or `128` = `328.125 KBits/s`)* | PS2 max is `500 kHz`; `2.625 MHz` is >5× too fast |
| **Clock Polarity (CPOL)** | `Low` | **`High`** | SPI Mode 3: `CLK` idles HIGH |
| **Clock Phase (CPHA)** | `1 Edge` | **`2 Edge`** | SPI Mode 3: shifts on falling edge, samples on rising edge |

### 3.2 Optional CubeMX GPIO Pull-Up on `PC11` (`SPI3_MISO`)
- In CubeMX **GPIO → SPI3**: set `PC11` (`SPI3_MISO`) Pull-up/Pull-down to **`Pull-up`** (still keep the external **4.7 kΩ – 10 kΩ** pull-up resistor on `DAT` → `3.3V` for reliable fast edges).
- `PD4` (`SPI3_CS1`) is already `GPIO_Output`, `PinState = GPIO_PIN_SET`, `No pull`, `Low speed` — no change needed (or rename its User Label to `PS2_ATT` if you want).

### 3.3 Exact Diff in `MM_control.ioc` (and Generated `Core/Src/spi.c`)

When you make the changes above in CubeMX and save, the `SPI3.*` block in `STM32_FIRMWARE/MM_control.ioc` changes from:
```ini
SPI3.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16
SPI3.CalculateBaudRate=2.625 MBits/s
SPI3.Direction=SPI_DIRECTION_2LINES
SPI3.IPParameters=VirtualType,Mode,Direction,CalculateBaudRate,BaudRatePrescaler
SPI3.Mode=SPI_MODE_MASTER
SPI3.VirtualType=VM_MASTER
```
to:
```ini
SPI3.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256
SPI3.CLKPhase=SPI_PHASE_2EDGE
SPI3.CLKPolarity=SPI_POLARITY_HIGH
SPI3.CalculateBaudRate=164.0625 KBits/s
SPI3.Direction=SPI_DIRECTION_2LINES
SPI3.FirstBit=SPI_FIRSTBIT_LSB
SPI3.IPParameters=VirtualType,Mode,Direction,CalculateBaudRate,BaudRatePrescaler,FirstBit,CLKPolarity,CLKPhase
SPI3.Mode=SPI_MODE_MASTER
SPI3.VirtualType=VM_MASTER
```

After **Generate Code** in CubeMX, verify `MX_SPI3_Init()` in `STM32_FIRMWARE/Core/Src/spi.c` matches:
```c
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
```

---

## 4. PS2 Protocol Reference

### 4.1 Timing Summary
- **SPI Mode 3** (`CPOL = 1`, `CPHA = 1`), **LSB First**, **164.06 kHz** (`Prescaler 256`).
- **Packet Framing:** Pull `ATT` (`PD4`) LOW → wait ~15 µs → transfer bytes **one-by-one** with **20 µs delay between each byte** → pull `ATT` HIGH → wait ~15 µs.
- **Do NOT burst all 9 bytes in one `HAL_SPI_TransmitReceive(..., 9, ...)` call** — the PS2 RX board needs ~12–20 µs between bytes to prepare the next response byte.
- **Polling Rate:** Poll continuously at **20–50 Hz** (every `20 ms` in FreeRTOS). Wireless controllers revert to Digital mode (`0x41`) or sleep if not polled within ~1 second.

### 4.2 Packet Header (First 3 Bytes of Every Transaction)

| Byte # | Index | CMD (STM32 → RX) | DAT (RX → STM32) | Meaning |
|---:|---:|---|---|---|
| **1** | `[0]` | `0x01` | `0xFF` | Start of packet |
| **2** | `[1]` | `<cmd>` | `<mode>` | **CMD:** Command ID (`0x42`, `0x43`, etc.)<br>**DAT:** Device Mode: `0x41` = Digital (5 B), `0x73` = Analog (9 B), `0x79` = Analog + Pressure (21 B), `0xF3` = Config (9 B) |
| **3** | `[2]` | `0x00` | `0x5A` | Data-ready marker (`0x5A` confirms valid communication) |

### 4.3 Digital Button Mapping (Bytes 4 & 5 — Active LOW)

Raw bit = `0` when **pressed**, `1` when **released** (`0xFF` = nothing pressed).

| Bit | Byte 4 (`rx[3]`) | Byte 5 (`rx[4]`) |
|---:|---|---|
| **0 (LSB)** | `Select` | `L2` |
| **1** | `L3` (Left stick click) | `R2` |
| **2** | `R3` (Right stick click) | `L1` |
| **3** | `Start` | `R1` |
| **4** | `Up` | `Triangle` |
| **5** | `Right` | `Circle` (`O`) |
| **6** | `Down` | `Cross` (`X`) |
| **7 (MSB)** | `Left` | `Square` |

### 4.4 Analog Joysticks & Pressures (Bytes 6–21)

| Byte # | Index | Signal | Value Range |
|---:|---:|---|---|
| **6** | `rx[5]` | **RX** (Right Stick X) | `0x00` (left) — `0x7F` (center) — `0xFF` (right) |
| **7** | `rx[6]` | **RY** (Right Stick Y) | `0x00` (up) — `0x7F` (center) — `0xFF` (down) |
| **8** | `rx[7]` | **LX** (Left Stick X) | `0x00` (left) — `0x7F` (center) — `0xFF` (right) |
| **9** | `rx[8]` | **LY** (Left Stick Y) | `0x00` (up) — `0x7F` (center) — `0xFF` (down) |
| **10–21** | `rx[9..20]` | **Button Pressures** *(only in `0x79` mode)* | Order: `Right, Left, Up, Down, Tri, O, X, Sqr, L1, R1, L2, R2` (`0x00` = released, `0xFF` = full) |

### 4.5 Initialization & Polling Byte Sequences

| Step | Purpose | Command Bytes (STM32 → RX) | Expected Reply (RX → STM32) |
|---:|---|---|---|
| **1** | Short poll (wake-up) | `01 42 00 00 00` | `FF 41 5A FF FF` |
| **2** | Enter Config Mode (`0x43`) | `01 43 00 01 00` | `FF 41 5A FF FF` |
| **3** | Enable & Lock Analog Mode (`0x44`) | `01 44 00 01 03 00 00 00 00` | `FF F3 5A 00 00 00 00 00 00` |
| **4** | Enable Vibration Motor Mapping (`0x4D`) | `01 4D 00 00 01 FF FF FF FF` | `FF F3 5A 00 01 FF FF FF FF` |
| **5** | Enable Pressures (`0x4F`) *(optional)* | `01 4F 00 FF FF 03 00 00 00` | `FF F3 5A 00 00 00 00 00 5A` |
| **6** | Exit Config Mode (`0x43`) | `01 43 00 00 5A 5A 5A 5A 5A` | `FF F3 5A 00 00 00 00 00 00` |
| **7** | **Loop Poll (`0x42`)** | `01 42 00 WW YY 00 00 00 00` *(9 B, or 21 B with step 5)* | `FF 73 5A ...` *(or `FF 79 5A ...`)* |

- `WW` (`tx[3]`): Small vibration motor (`0xFF` = ON, `0x00` = OFF).
- `YY` (`tx[4]`): Large vibration motor intensity (`0x00` = OFF, `0x40`–`0xFF` = spinning).

---

## 5. Firmware Implementation for `STM32_FIRMWARE` (`SPI3`)

Follow the existing `tue_*.h` / `tue_*.c` convention in `STM32_FIRMWARE/Core/`.

### 5.1 Header File — `Core/Inc/tue_ps2.h`

```c
#ifndef TUE_PS2_H
#define TUE_PS2_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Active-HIGH button masks (1 = pressed) */
#define TUE_PS2_BTN_SELECT    (1U << 0)
#define TUE_PS2_BTN_L3        (1U << 1)
#define TUE_PS2_BTN_R3        (1U << 2)
#define TUE_PS2_BTN_START     (1U << 3)
#define TUE_PS2_BTN_UP        (1U << 4)
#define TUE_PS2_BTN_RIGHT     (1U << 5)
#define TUE_PS2_BTN_DOWN      (1U << 6)
#define TUE_PS2_BTN_LEFT      (1U << 7)
#define TUE_PS2_BTN_L2        (1U << 8)
#define TUE_PS2_BTN_R2        (1U << 9)
#define TUE_PS2_BTN_L1        (1U << 10)
#define TUE_PS2_BTN_R1        (1U << 11)
#define TUE_PS2_BTN_TRIANGLE  (1U << 12)
#define TUE_PS2_BTN_CIRCLE    (1U << 13)
#define TUE_PS2_BTN_CROSS     (1U << 14)
#define TUE_PS2_BTN_SQUARE    (1U << 15)

typedef struct {
    bool     connected;      /* true when rx[2] == 0x5A */
    uint8_t  mode;           /* 0x41 digital, 0x73 analog, 0x79 analog+pressure */
    uint16_t buttons;        /* Current active-HIGH button bitmask */
    uint16_t buttons_prev;   /* Previous poll bitmask (for edge detection) */
    uint8_t  rx, ry;         /* Right stick (0..255, center ~128) */
    uint8_t  lx, ly;         /* Left stick  (0..255, center ~128) */
} tue_ps2_state_t;

void tue_ps2_init(void);
bool tue_ps2_poll(tue_ps2_state_t *out, bool small_motor, uint8_t large_motor);

static inline bool tue_ps2_pressed(const tue_ps2_state_t *s, uint16_t mask) {
    return (s->buttons & mask) != 0U;
}
static inline bool tue_ps2_just_pressed(const tue_ps2_state_t *s, uint16_t mask) {
    return ((s->buttons & ~s->buttons_prev) & mask) != 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* TUE_PS2_H */
```

### 5.2 Source File — `Core/Src/tue_ps2.c`

Uses **`hspi3`** and **`SPI3_CS1_GPIO_Port` / `SPI3_CS1_Pin` (`PD4`)** already defined in `Core/Inc/main.h`, plus the Cortex-M4 **DWT cycle counter** (`168 MHz`) for microsecond inter-byte delays.

```c
#include "tue_ps2.h"
#include "main.h"
#include "spi.h"

/* Uses PD4 (SPI3_CS1) from main.h; change to SPI3_CS2_Pin (PD3) or SPI3_CS3_Pin (PD2) if desired */
#define PS2_ATT_PORT      SPI3_CS1_GPIO_Port
#define PS2_ATT_PIN       SPI3_CS1_Pin

#define PS2_BYTE_DELAY_US 20U
#define PS2_ATT_DELAY_US  15U

static void dwt_delay_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

static void ps2_transact(const uint8_t *tx, uint8_t *rx, uint8_t len) {
    HAL_GPIO_WritePin(PS2_ATT_PORT, PS2_ATT_PIN, GPIO_PIN_RESET);
    delay_us(PS2_ATT_DELAY_US);

    for (uint8_t i = 0; i < len; i++) {
        uint8_t tx_b = tx[i];
        uint8_t rx_b = 0xFF;
        HAL_SPI_TransmitReceive(&hspi3, &tx_b, &rx_b, 1, 5);
        rx[i] = rx_b;
        delay_us(PS2_BYTE_DELAY_US);
    }

    HAL_GPIO_WritePin(PS2_ATT_PORT, PS2_ATT_PIN, GPIO_PIN_SET);
    delay_us(PS2_ATT_DELAY_US);
}

static void ps2_configure_analog(void) {
    uint8_t rx[9];

    const uint8_t cmd_poll[5]   = {0x01, 0x42, 0x00, 0x00, 0x00};
    const uint8_t cmd_enter[5]  = {0x01, 0x43, 0x00, 0x01, 0x00};
    const uint8_t cmd_analog[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    const uint8_t cmd_motors[9] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t cmd_exit[9]   = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

    ps2_transact(cmd_poll, rx, sizeof(cmd_poll));
    delay_us(200);
    ps2_transact(cmd_enter, rx, sizeof(cmd_enter));
    delay_us(200);
    ps2_transact(cmd_analog, rx, sizeof(cmd_analog));
    delay_us(200);
    ps2_transact(cmd_motors, rx, sizeof(cmd_motors));
    delay_us(200);
    ps2_transact(cmd_exit, rx, sizeof(cmd_exit));
}

void tue_ps2_init(void) {
    dwt_delay_init();
    HAL_GPIO_WritePin(PS2_ATT_PORT, PS2_ATT_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    ps2_configure_analog();
}

bool tue_ps2_poll(tue_ps2_state_t *out, bool small_motor, uint8_t large_motor) {
    uint8_t tx[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[9] = {0};

    tx[3] = small_motor ? 0xFFU : 0x00U;
    tx[4] = large_motor;

    ps2_transact(tx, rx, sizeof(tx));

    if (rx[2] != 0x5AU) {
        out->connected = false;
        out->mode      = rx[1];
        return false;
    }

    /* Re-enable Analog if handheld woke up or reconnected in Digital mode (0x41) */
    if (rx[1] == 0x41U) {
        ps2_configure_analog();
    }

    out->connected    = true;
    out->mode         = rx[1];
    out->buttons_prev = out->buttons;
    out->buttons      = (uint16_t)((uint8_t)(~rx[3]) | ((uint16_t)(uint8_t)(~rx[4]) << 8));

    if ((rx[1] & 0xF0U) == 0x70U) {
        out->rx = rx[5];
        out->ry = rx[6];
        out->lx = rx[7];
        out->ly = rx[8];
    } else {
        out->rx = 0x7FU;
        out->ry = 0x7FU;
        out->lx = 0x7FU;
        out->ly = 0x7FU;
    }
    return true;
}
```

### 5.3 Integrating Into `STM32_FIRMWARE`

1. **`STM32_FIRMWARE/CMakeLists.txt`** — Add `Core/Src/tue_ps2.c` under `target_sources(${CMAKE_PROJECT_NAME} PRIVATE ...)`:
   ```cmake
   Core/Src/tue_ps2.c
   ```
2. **`STM32_FIRMWARE/Core/Src/main.c`** — `MX_SPI3_Init()` is already called at line 106 before `osKernelStart()`.
3. **`STM32_FIRMWARE/Core/Src/freertos.c`** — Poll `tue_ps2_poll()` every `20 ms` (50 Hz) in `StartDefaultTask` or a dedicated FreeRTOS task:
   ```c
   #include "tue_ps2.h"

   static tue_ps2_state_t g_ps2 = {0};

   /* Inside StartDefaultTask or a dedicated PS2Task (stack >= 256 words): */
   tue_ps2_init();

   for (;;) {
       if (tue_ps2_poll(&g_ps2, false, 0)) {
           int16_t stick_lx = (int16_t)g_ps2.lx - 128; /* -128 .. +127 */
           int16_t stick_ly = (int16_t)g_ps2.ly - 128;

           if (tue_ps2_just_pressed(&g_ps2, TUE_PS2_BTN_CROSS)) {
               /* Handle Cross button press */
           }
           (void)stick_lx;
           (void)stick_ly;
       } else {
           /* Fail-safe: stop motors if controller disconnects */
       }
       osDelay(20);
   }
   ```

### 5.4 Implemented Driver & Host Interface (as built)

The firmware now ships `Core/Inc/tue_ps2.h` / `Core/Src/tue_ps2.c`. `tue_ps2_init()`
is called from `tue_usb_init()`, starts a `tue_ps2` FreeRTOS task and configures
the handheld for analog mode. The task polls every 20 ms (50 Hz), re-runs the
analog configuration if the link drops or the handheld reverts to digital, and
publishes a thread-safe snapshot plus the two rumble motor values.

USB protocol IDs (see `Core/Inc/tue_usb_binary.h`):

| CAN ID | Name | Request | Response |
|---|---|---|---|
| `0x505` | `PS2_CTRL` | `DLC=2`: `[small motor (0/1), large motor (0..255)]`, or `DLC=0` to query | 4 B: `[connected, mode, small, large]` |
| `0x506` | `PS2_STATE` | `DLC=0` | 8 B: `[buttons u16 LE, lx, ly, rx, ry, mode, flags]` — `flags` bit0 = connected, bit1 = analog sticks valid |

Python host library (`STM32_FIRMWARE/python/tue_usb`):

```python
from tue_usb import TueUsbClient, Ps2Buttons

with TueUsbClient("COM7") as ps2:
    ps2.ps2_set_motors(small_motor=True, large_motor=0x80)
    state = ps2.ps2_get_state()
    if state.pressed(Ps2Buttons.CROSS):
        print("cross", state.lx, state.ly)
```

Demo: `python run_gui.py --demo` opens the GUI already connected to the mock
engine on the `PS2 (SPI3)` tab. `python run_gui.py --tab ps2` uses real hardware.

---

## 6. Wireless RX Board Troubleshooting (`SPI3`)

| Symptom | Likely Cause & Fix |
|---|---|
| `rx` is always `0xFF 0xFF 0xFF ...` | Missing pull-up on `DAT` (`PC11`) → add external **4.7 kΩ–10 kΩ** to `3.3V`. Verify `VCC` is **3.3 V** and `PD4` (`SPI3_CS1`) toggles LOW during transfer. |
| RX board LED blinks, never pairs | Turn on the PS2 handheld controller and press **START** or **ANALOG**; check handheld batteries. |
| `rx[2]` is not `0x5A` (garbled bytes) | Check `Core/Src/spi.c` `MX_SPI3_Init()`: **`SPI_POLARITY_HIGH`**, **`SPI_PHASE_2EDGE`**, **`SPI_FIRSTBIT_LSB`**, **`SPI_BAUDRATEPRESCALER_256`**. Increase `PS2_BYTE_DELAY_US` to `30–50`. |
| `tue_spi_adc` interferes on `SPI3` | Not possible any more: `tue_spi_adc` is SPI1-only and rejects `bus_id != 1`. If you see `PD4` toggling, you are running old firmware. |
| Joysticks drift around center (`125..131`) | Apply a software deadzone (e.g., `abs(stick) < 10 ? 0 : stick`) before commanding DC/stepper motors or servos. |
