# Haptic Communication & Protocol Reference

End-to-end protocol documentation for the tactile-to-pneumatic haptic link, covering all
three subsystems: **CONTROL_GUI** (host PC), **STM32_FIRMWARE** (master/bridge), and
**ESP32_FIRMWARE** (haptic slave). Applies to the current system state: ESP32 C++ port
(`ESP32_FIRMWARE/src/*.cpp`) with `src/main.py` kept as the MicroPython reference.

---

## 1. System overview

```
 Host PC (CONTROL_GUI)                STM32F407 (master)              ESP32-WROOM (slave)
+---------------------------------+  +-------------------------+  +------------------------------+
| HapticPayloadSynthesizer        |  | tue_usb_binary.c        |  | tue_protocol.cpp             |
|   52x2 taxels -> 8-byte payload |  |  usb_dispatch()         |  |  Serial2 sliding-window     |
| TueBinaryEncoder                |  |   0x300: validate +    |  |  parser (SOF/CRC/EOF)        |
|   8-byte payload -> 20-byte TUE |  |     forward verbatim    |  | haptic_core.cpp             |
| Stm32HapticStreamer             |  |   0x301/0x302: request |  |  forces -> 5 valve voltages |
|   100 Hz writer thread          |  |     <-> ESP32           |  |  PCA9548A -> 5x MCP4725     |
+---------------|-----------------+  +-------------|-----------+  +-------------> 5 valves A-E
        USB CDC (binary TUE frames)        USART2 115200 8-N-1
        (motor + haptic share 1 COM port)  (polling, no IRQ/DMA)
```

Three physical links, one shared 20-byte wire format:

| Link | Physical | Direction | Purpose |
|---|---|---|---|
| GUI ↔ STM32 | USB CDC (STM32 COM port, shared with motor control) | GUI → STM32 (0x300 @ 100 Hz), GUI ← → STM32 (0x301/0x302) | haptic telemetry stream, ping, link diag |
| STM32 ↔ ESP32 | USART2: STM32 PD5 (TX) → ESP32 GPIO16 (UART2 RX), STM32 PD6 (RX) ← ESP32 GPIO17 (TX), common GND, 115200 8-N-1 | STM32 → ESP32 (forwarded frames), ESP32 → STM32 (0x301 reply, optional 0x303) | valve control + slave status |
| ESP32 → valves | I2C (SDA 21 / SCL 22, 400 kHz) via PCA9548A/TCA9548A mux @ 0x77 | ESP32 → 5x MCP4725 DACs | pneumatic chamber pressures |

---

## 2. Wire frame format (identical on USB and UART)

All multi-byte fields are **little-endian**. Total frame length is fixed at 20 bytes
(12 bytes framing + 8 bytes payload).

```
+------+------+------+-----------+-----+-----+-----------+------+------+------+
| Byte |  0   |  1   |  2        | 3..6        |  7  |  8  | 9..16     | 17   | 18   | 19   |
|      | SOF1 | SOF2 | TYPE      | CAN_ID (LE)| DLC | SEQ | PAYLOAD   | CRC8 | EOF1 | EOF2 |
|      | 0xAA | 0x55 | u8        | u32 LE     | 0x08| u8  | 8 bytes  |      | 0x55 | 0xAA |
+------+------+------+-----------+------------+-----------+-----------+------+------+------+
```

**CRC-8 (SAE J1850)** — computed over bytes 2..16 (TYPE + CAN_ID + DLC + SEQ + PAYLOAD = 15 bytes):
- Poly `0x1D`, Init `0xFF`, XorOut `0xFF`, MSB-first, no reflection
- Check vector: `crc8("123456789") == 0x4B` (self-tested at boot in STM32, ESP32 and GUI)

**Frame types** (byte 2): `0x01` REQ (host→MCU / master→slave), `0x02` RESP_OK,
`0x03` RESP_ERR, `0x04` ASYNC.

**CAN ID allocation** (IDs `0x300`–`0x3FF` are reserved for user custom traffic;
`TueUsbClient.transact` refuses them, so haptic frames are written raw):

| CAN ID | Name | Type | Direction | Purpose |
|---|---|---|---|---|
| `0x300` | Haptic telemetry | REQ | GUI → STM32 → ESP32 | 8-byte valve payload, 100 Hz, fire-and-forget (no USB ACK — the host must not be throttled) |
| `0x301` | Haptic ping | REQ → RESP_OK | GUI → STM32 → ESP32 → back | slave liveness + status; token/seq echoed |
| `0x302` | UART link diag | REQ → RESP_OK/ERR | GUI → STM32 | USART2 link statistics (STM32-local, ESP32 not involved) |
| `0x303` | Beacon | ASYNC | ESP32 → STM32 | optional 1 Hz heartbeat; **default OFF** (STM32 RX is polling-only, unsolicited bytes become overruns); manual via ESP32 console `>T<`/`>B<` |

---

## 3. CAN 0x300 — haptic telemetry payload (8 bytes)

Synthesized by the GUI from the two 52-taxel force-sensor jaws; consumed by the ESP32.

| Byte | Field | Unit / Range | Meaning | Drives |
|---|---|---|---|---|
| 0 | `Fz_Left` | 0.1 N/LSB, 0..25.5 N | left jaw normal force | Valve D (lateral left) |
| 1 | `Fz_Right` | 0.1 N/LSB, 0..25.5 N | right jaw normal force | Valve E (lateral right) |
| 2 | `F_shear_avg` | 0.1 N/LSB, 0..25.5 N | average of the two jaws' tangential shear | Valves A/B/C via weights |
| 3 | `Weight_Tip` | 0..255 → 0.0..1.0 | zone A share (taxel rows 0–15) | Valve A (bottom – distal tip) |
| 4 | `Weight_Mid` | 0..255 → 0.0..1.0 | zone B share (taxel rows 16–35) | Valve B (bottom – medial mid) |
| 5 | `Weight_Base` | 0..255 → 0.0..1.0 | zone C share = `255 − W_A − W_B` (taxel rows 36–51) | Valve C (bottom – proximal base) |
| 6 | `Status_Flags` | bitmask (below) | contact/slip/tare status | display + STM32 slip counter |
| 7 | `Frame_Counter` | 0..255, rolling | per-frame sequence from the GUI | watchdog/debug (echoed in ping reply) |

**Status_Flags bitmask** (byte 6):

| Bit | Mask | Meaning |
|---|---|---|
| 0 | `0x01` | Left jaw contact (fz > 0.20 N, fresh data) |
| 1 | `0x02` | Right jaw contact |
| 2 | `0x04` | Slip alert (shear/fz > µ = 0.50, or explicit GUI alert) |
| 3 | `0x08` | Tare active |

### 3.1 GUI synthesis (`CONTROL_GUI/unified_gui/haptic.py`)

- Inputs per jaw: `fz`, `fy` (or `fx`, shear axis configurable, default **Y**), `taxels_fz` (52-vector or (52,3) array), optional `t`/`age_s`.
- Stale-data guard: a jaw whose last frame is older than 0.10 s contributes zeros.
- Forces are quantized with `clamp_u8`: `byte = clamp(0, 255, round(N * 10))`.
- Zone weights: sum both jaws' 52-taxel Fz vectors; if total > 0.20 N,
  `W_A = 255·Σ(zone tip)/Σtotal`, `W_B = 255·Σ(zone mid)/Σtotal`, `W_C = 255 − W_A − W_B`; else all zero.
- `Frame_Counter` is the synthesizer's rolling 8-bit counter (independent of the frame header `SEQ`).
- Acceptance vectors (GUI `self_test()`): pinch 5 N / 3 N → `32 1E 00 00 00 00 03 xx`; tip-only shear 2 N/1 N → `14 14 0A FF 00 00 03 xx`.

### 3.2 STM32 handling (`Core/Src/tue_usb_binary.c`, `Core/Src/tue_haptic_bridge.c`)

- `usb_dispatch()` accepts 0x300 only as `TYPE=REQ, DLC=8` raw 20 bytes → **no USB response** (fire-and-forget, host stays at 100 Hz).
- `tue_haptic_ingest()` re-validates SOF/EOF/TYPE/CAN_ID/DLC/CRC, caches the payload atomically (`g_tue_haptic_payload`, PRIMASK-guarded, available for reflex control), counts SLIP_ALERTs, then forwards the **exact same 20 raw bytes** to USART2 with a blocking polling TX (20 ms timeout; failures count `tx_drops`).

### 3.3 ESP32 handling (`ESP32_FIRMWARE/src/tue_protocol.cpp`)

- Sliding-window parser over a 512 B buffer: preamble/postamble/CRC validated; resync drops 1 byte on mismatch; valid 0x300 + DLC=8 frames reset the error counter and are dispatched to `handle_haptic_packet()`.
- Valves: `f_A = F_shear·(W_A/255)`, `f_B = F_shear·(W_B/255)`, `f_C = F_shear·(W_C/255)`, `f_D = Fz_Left`, `f_E = Fz_Right` (all in N).
- Each force goes through the transfer function (§6.1); DACs written only when voltage changed ≥ 0.015 V.

---

## 4. CAN 0x301 — binary ping

**Request** (GUI → STM32 over USB as normal REQ; STM32 → ESP32 over UART, `TYPE=0x01`):
the request frame carries the host-chosen `SEQ` and a caller-chosen token byte in `payload[0]`.
STM32 drains USART2 RX first, then sends the request and polls for the reply with up to
**2 attempts × 90 ms** (`TUE_HAPTIC_PING_TIMEOUT_MS/ATTEMPTS`), scanning a sliding window
(noise-tolerant). The reply must be a fully valid 20-byte frame: `TYPE=0x02`,
`CAN_ID=0x301`, `DLC=8`, **matching SEQ**, **token echoed at payload[0]**, valid CRC.
On success the STM32 relays the 8-byte slave payload to the host in a USB RESP_OK;
on timeout the host gets USB RESP_ERR with error code `0x0A` (timeout).

**Reply payload (ESP32 → STM32 → GUI)**, built by `build_ping_reply()`:

| Byte | Field | Meaning |
|---|---|---|
| 0 | Token | echo of the request token |
| 1 | Protocol version | `0x01` |
| 2 | FW major | `0x01` (ESP32 firmware 1.0) |
| 3 | FW minor | `0x00` |
| 4 | System state | 0 = IDLE, 1 = RUNNING, 2 = ERROR |
| 5 | Last haptic frame counter | last 0x300 payload byte 7 received |
| 6 | Consecutive errors | ESP32 UART parse errors, saturated at 255 |
| 7 | Uptime | seconds since ESP32 boot, mod 256 |

The ping does **not** refresh the ESP32 haptic watchdog and does not touch valve outputs.
Decoded on the host by `tue_usb.payloads.HapticPingInfo` (display only — no version gate).
Triggered from the GUI haptic panel button "Ping ESP32 (UART 115200)".

---

## 5. CAN 0x302 — STM32-local UART diagnostics (ESP32 not involved)

8-byte RESP_OK payload from `tue_haptic_get_diag()` (`Core/Src/tue_haptic_bridge.c`):

| Byte | Meaning |
|---|---|
| 0 | flags: bit0 link initialized, bit1 RX ready (polling), bit3 last TX ok, bit4 last ping saw RX bytes |
| 1 | USART2 RX error events (ORE/FE/NE/PE), mod 256 |
| 2..3 | total RX bytes from ESP32, LE16 |
| 4 | RX bytes harvested during the last ping, mod 256 |
| 5 | ping TX failures |
| 6 | ping timeouts |
| 7 | ping replies received |

`0x302` also harvests any byte sitting in the USART2 DR before reporting (beacons count as RX traffic).
Decoded by `tue_usb.payloads.HapticUartDiag`.

---

## 6. ESP32 haptic core (data → valve ports)

### 6.1 Transfer function (`compute_valve_voltage`)

```
if force <= 0.20 N (F_THRESH):        V = 0.0          (noise deadband)
ratio = clamp((force − 0.20) / 19.8)                    (F_RATED = 20.0 N)
V = 0.70 + (2.20 − 0.70) · ratio                        (V_MIN .. V_MAX)
V clamped to ≤ 2.20 V                                   (absolute safety limit — 3.3V never possible)
```

### 6.2 Output hardware map

| Valve | Mux ch | MCP4725 addr | Position |
|---|---|---|---|
| A | 0 | 0x60 | bottom pad – distal tip |
| B | 1 | 0x60 | bottom pad – medial mid |
| C | 2 | 0x60 | bottom pad – proximal base |
| D | 3 | 0x61 | lateral left flank |
| E | 4 | 0x61 | lateral right flank |

I2C: SDA 21 / SCL 22 @ 400 kHz; PCA9548A/TCA9548A mux @ 0x77 (A0/A1/A2 = 27/26/25 all high,
reset pin 18). DAC write: `0x40, val>>4, (val&0x0F)<<4` with `val = (V/3.30)·4095`.
The mux channel register is `1 << ch`; all channels are disconnected when idle.

### 6.3 ESP32 failsafes (all layers below force 0.0 V)

| Condition | Response |
|---|---|
| Power-up (before anything else runs) | all 5 DACs written to 0 V through the mux (`safe_boot_vent`), mux channels disabled — outputs stay 0 V until valid 0x300 data arrives |
| No valid 0x300 frame > 150 ms while RUNNING | vent all chambers → IDLE |
| Any channel > 0 V while not RUNNING | forced to 0 V every loop pass |
| Postamble/CRC mismatch | drop 1 byte (resync) + count error; 5 consecutive → vent + ERROR |
| > 100 garbage bytes in one parse pass | clear buffer + count error (valid frames parsed first) |
| Incomplete < 20 B fragment idle > 25 ms | discarded |
| I2C write failure while updating a valve | vent + ERROR |
| Leaving RUNNING state | auto-vent |
| Exiting ERROR | requires 3 consecutive good 0x300 frames before re-pressurizing |
| ESP32 boot self-test failure | vent + red LED + halt |

### 6.4 ESP32 system states (reported in ping reply byte 4)

| State | Green LED | Blue LED | Red LED | Valves |
|---|---|---|---|---|
| IDLE (0) | ON | off | off | 0 V |
| RUNNING (1) | off | 1.6 s breathing PWM | off | follows 0x300 data |
| ERROR (2) | off | off | 100/70 ms blink | 0 V (vent loop) |

### 6.5 ESP32 USB debug console (optional, non-blocking; 115200 baud)

| Command | Action |
|---|---|
| `>G<` | print the 5 cached DAC voltages (`%.3f`, V) |
| `>R<` | vent all chambers, print `RESET_OK`, go IDLE |
| `>S<` | per-mux-channel I2C scan (`ch:AA,BB` or `ch:-`, excludes 0x77) |
| `>P<` | `PING_STATS uart_frames=… rx_bytes=… ping_rx=… ping_tx=… state=… errors=… beacon=… beacon_tx=…` |
| `>U<` | UART2 config + same counters |
| `>T<` | send one 0x303 beacon immediately |
| `>B<` | toggle the 1 Hz beacon (default OFF) |

Boot banner: `HAPTIC_SLAVE_READY (UART2 @ 115200 baud, Binary 20B 0x300/0x301, Anti-Error Failsafe Active)`.

---

## 7. CAN 0x303 — ESP32 beacon (optional, default OFF)

20-byte ASYNC frame (`TYPE=0x04`) sent once per second when enabled (`>B<`):

| Byte | Field | Byte | Field |
|---|---|---|---|
| 0 | `0xBE` magic | 4 | FW major |
| 1 | `0xAC` magic | 5 | FW minor |
| 2 | protocol version (1) | 6 | system state |
| 3 | — | 7 | uptime mod 256 |

The STM32 has **no 0x303 handler** — beacons are only harvested as RX-traffic counters
by `uart_drain_rx()` (visible indirectly via 0x302 byte 4). Keep the beacon disabled during
normal operation so the STM32's single-byte polling RX never overruns.

---

## 8. Timing summary

| Parameter | Value | Where |
|---|---|---|
| Haptic telemetry rate | 100 Hz host-paced (10 ms period; GUI raises Windows timer to 1 ms) | GUI streamer → whole chain |
| 0x300 USB behavior | fire-and-forget, no ACK | STM32 `usb_dispatch` |
| STM32 → ESP32 UART | 115200 8-N-1, polling only (USART2 IRQ disabled, no DMA) | both sides |
| Frame wire time | 20 B × 10 bits ≈ 1.74 ms @ 115200 (~17 % of a 10 ms period) | UART link |
| STM32 UART TX timeout | 20 ms per frame | `uart_write_bytes` |
| Ping budget | 90 ms × 2 attempts, reply must be a valid RESP_OK frame | `tue_haptic_ping` |
| ESP32 watchdog | 150 ms without valid 0x300 → vent | `check_watchdog` |
| GUI stale-jaw guard | 0.10 s → jaw contributes zeros | `HapticPayloadSynthesizer` |
| GUI streamer give-up | 50 consecutive write errors | `Stm32HapticStreamer` |
| GUI standalone baud | 1,000,000 (bench/tests; in the GUI the haptic stream shares the motor client's STM32 COM port — one port per physical cable) | `Stm32HapticStreamer` |

---

## 9. Implementation file map

| Layer | Files |
|---|---|
| GUI | `CONTROL_GUI/unified_gui/haptic.py` (synthesizer / encoder / 100 Hz streamer), `CONTROL_GUI/unified_gui/panels/haptic.py` (panel + ping button), `CONTROL_GUI/unified_gui/valvemap.py`, `CONTROL_GUI/tests/test_haptic.py` (acceptance vectors) |
| Host library (shared) | `STM32_FIRMWARE/python/tue_usb/` (`frame.py`, `crc.py`, `client.py` `send_haptic_telemetry()`/`haptic_ping()`/`haptic_uart_diag()`, `payloads.py` decoders) |
| STM32 | `Core/Src/tue_usb_binary.c` (USB protocol task, `tue_usb_crc8`), `Core/Src/tue_haptic_bridge.c` + `Core/Inc/tue_haptic_bridge.h` (USART2 bridge, ping, diag), `Core/Src/usart.c` (USART2 115200 config), `docs/USB_BINARY_PROTOCOL.md` (base TUE spec) |
| ESP32 (C++) | `ESP32_FIRMWARE/src/main.cpp` (boot self-test, loop, debug console), `src/tue_protocol.cpp` + `include/tue_protocol.h` (CRC, frames, parser, ping, beacon), `src/haptic_core.cpp` + `include/haptic_core.h` (mux + DACs, state machine, watchdog), `include/config.h` (all constants), `ESP32_CPP_PORT_REPORT.md` (port report) |
| ESP32 (legacy reference) | `ESP32_FIRMWARE/src/main.py` (MicroPython original — kept for reference) |
| Spec / tasks | `CONTROL_GUI/TODO_SEND_SENSOR_DATA_THROUGH_USB.md` (original haptic spec), `STM32_FIRMWARE/USB_STM32_UPDATE.md` (historical 250 kbaud DMA design — superseded by current 115200 polling), `STM32_FIRMWARE/USB_UART_IMPLEMENTATION_REPORT.md` |

> Note: `docs/USB_BINARY_PROTOCOL.md` §5.2 still describes an older ASCII `PING`/`PONG`
> UART ping; the actual implemented ping is the binary 0x301 transaction described in
> §4 above. Wire constants and CRC are unchanged.

---

## 10. Verification vectors

| Check | Expected | Verified by |
|---|---|---|
| CRC-8 SAE J1850 of `"123456789"` | `0x4B` | GUI `crc8_selftest()`, STM32 `tue_usb_crc8_selftest()`, ESP32 boot self-test |
| 20-byte frame structure | SOF `AA 55`, EOF `55 AA`, DLC 8, CRC at byte 17 | all three implementations + STM32 `_Static_assert`s |
| GUI pinch 5 N left / 3 N right | payload starts `32 1E 00 00 00 00 03` | `tests/test_haptic.py` |
| GUI tip-zone shear 2 N / 1 N | payload starts `14 14 0A FF 00 00 03` | `tests/test_haptic.py` |
| ESP32 ping reply | TYPE `0x02`, CAN 0x301, SEQ + token echoed, valid CRC | ESP32 boot self-test + GUI "Ping ESP32 (UART 115200)" round-trip |
| Power-up safety | all valves 0.00 V until first valid 0x300 frame | ESP32 `>G<` after boot |
