# ESP32 C++ Port Implementation Report

**Task:** Port the ESP32 haptic slave controller from MicroPython (`src/main.py`, 914 lines) to C++ on PlatformIO/Arduino, as a faithful 1:1 behavioral port.
**Date:** 2026-10-29
**Status:** Implemented, builds clean. On-hardware bench verification pending (checklist below).

---

## 1. What was done

The complete MicroPython firmware was rewritten in C++ for the existing PlatformIO
project (env `esp32dev`, ESP32-WROOM-32, Arduino framework, espressif32 6.10.0 /
arduino-esp32 2.0.17). All wire protocol, timing, failsafe and console behaviors are
identical to the working reference. The polling architecture is preserved
(no interrupts, no DMA), matching the STM32 master's polling USART2 driver.
`src/main.py` is kept as the reference (PlatformIO ignores `.py` files).

## 2. Module map (3 library modules + main)

| File | Role |
|---|---|
| `include/config.h` | All system configuration: pins, I2C addresses, UART, CAN IDs, frame constants, safety limits, failsafe timings, FW version. Header-only. |
| `include/tue_protocol.h` + `src/tue_protocol.cpp` | UART protocol module: SAE J1850 CRC-8 (table verbatim from main.py), 20-byte TUE frame builder, 0x301 ping reply builder, UART2 init (Serial2 @ 115200 8-N-1, GPIO16 RX / GPIO17 TX, 512 B RX buffer), sliding-window frame parser with anti-error protection, optional 0x303 beacon (default OFF), link statistics. |
| `include/haptic_core.h` + `src/haptic_core.cpp` | Haptic core module: PCA9548A/TCA9548A mux driver + 5x MCP4725 DAC driver (the data→port mapping), safety-first boot vent, failsafe `vent_all_chambers()`, state machine (IDLE/RUNNING/ERROR), 150 ms watchdog, haptic transfer function with V_MAX clamp, LED indicators (green solid / blue 1.6 s breathing / red blink), I2C scan helper. |
| `src/main.cpp` | Thin: `setup()`, free-running `loop()` (no delay), boot self-test, debug-only USB console. |

Dependency direction (no cycles): `config.h` → `haptic_core` → `tue_protocol` → `main`.

## 3. Ported configuration (identical to main.py)

| Group | Values |
|---|---|
| LEDs | green GPIO4, red GPIO13, blue GPIO14 (LEDC PWM ch0, 1 kHz, 16-bit), active LOW |
| I2C | SDA 21 / SCL 22 @ 400 kHz; PCA9548A/TCA9548A @ 0x77 (reset GPIO18, A0/A1/A2 = 27/26/25 high → addr 0x77) |
| DACs | MCP4725 ch0–2 @ 0x60, ch3–4 @ 0x61, write cmd 0x40, VDD 3.30, full scale 4095 |
| UART2 | 115200 8-N-1, TX GPIO17, RX GPIO16, RX buffer 512 B (from STM32 PD5/PD6) |
| Safety | V_MIN 0.70 V, V_MAX 2.20 V (absolute clamp), F_THRESH 0.20 N, F_RATED 20.0 N, write dedup ≥ 0.015 V |
| Failsafe | watchdog 150 ms, 5 consecutive errors → vent+ERROR, 3 recovery frames, stale fragment flush 25 ms, >100 garbage bytes/parse → flush |
| Identity | FW 1.0, ping protocol 1; CAN 0x300 telemetry, 0x301 ping, 0x303 beacon |

## 4. Safety-first boot (per review requirement)

`setup()` order:
1. `haptic_core_init()`: mux address pins high → `pca_reset_device()` (LOW 2 µs → HIGH → 2 ms) → `Wire.begin()` → **`safe_boot_vent()`: every MCP4725 is written to 0.0V through the mux (0x40/0x00/0x00 per channel, every channel attempted even if one fails), then all mux channels disabled** → LED init.
2. `Serial.begin()` (debug console), `tue_protocol_init()` (Serial2 + boot-glitch drain).
3. Boot self-test, `set_state(STATE_IDLE)`, `vent_all_chambers()`, boot banners.

Result: all valve ports are at 0 V from the first milliseconds after power-up and
**stay at 0 V until a fully valid 0x300 frame arrives over UART2**. Loss of signal,
CRC errors, wrong frames or I2C failures all force 0 V (see failsafe matrix below).

## 5. Failsafe matrix (all preserved 1:1)

| Condition | Response |
|---|---|
| No valid 0x300 frame for > 150 ms while RUNNING | vent all chambers → STATE_IDLE |
| Any channel > 0 V while not RUNNING | forced back to 0 V every loop pass |
| Postamble / CRC-8 mismatch | drop 1 byte (resync), count error; 5 consecutive → vent + STATE_ERROR |
| > 100 garbage bytes in one parse pass | clear buffer + count error (valid frames parsed first) |
| Incomplete < 20 B fragment idle for > 25 ms | discarded |
| I2C write failure while updating a valve | vent + STATE_ERROR |
| State transition out of RUNNING | auto-vent |
| Exiting STATE_ERROR | 3 consecutive good frames required before re-pressurizing |
| Self-test failure at boot | vent + red LED + halt |

## 6. Wire protocol (unchanged, verified against STM32 `tue_haptic_bridge.c`)

- 20-byte frame: `AA 55 | type | can_id LE32 | dlc=8 | seq | payload(8) | crc8 | 55 AA`; CRC-8 SAE J1850 over bytes 2..16 (poly 0x1D, init 0xFF, xorout 0xFF); self-test vector `"123456789"` → 0x4B.
- 0x300 payload: fz_left, fz_right, f_shear (0.1 N/LSB) → valves D/E and A/B/C via zone weights (tip/mid/base /255); forces mapped to 0.70–2.20 V through the deadband transfer function.
- 0x301 REQ answered synchronously with RESP_OK: token echo, protocol 1, FW 1.0, state, last frame counter, consecutive errors (saturated 255), uptime mod 256 — written to the wire **before** the console log line (STM32 budget: 90 ms x 2 attempts).
- 0x303 beacon: 1 Hz ASYNC, **default OFF** (STM32 RX is polling-only; unsolicited bytes become overruns), manual `>T<`/`>B<`.

## 7. USB console (debug-only, non-blocking)

Commands identical to the reference, output strings byte-identical:
`>G<` voltages, `>R<` vent+reset, `>S<` per-mux-channel I2C scan, `>P<`/`>U<` link stats, `>T<` one-shot beacon, `>B<` beacon toggle. Accepts `>X<` immediate framing or newline-terminated lines, 64-char line cap. Console processing is bounded (≤ 32 chars per loop pass), gated on `Serial.available()`, prints only on command response and boot banner — it cannot delay the UART2/valve path.

## 8. Translation decisions (Python → C++)

- `time.ticks_ms()/ticks_diff` → `millis()` with `(uint32_t)(now - last)` deltas (rollover-safe, same wrap semantics).
- Python `try/except` failsafe wrappers → boolean returns + explicit error paths (`Wire.endTransmission() != 0` replaces `OSError`); the reference's catch-all in `handle_haptic_packet` was provably unreachable (pure float arithmetic + fixed indices) and is covered by explicit I2C error checks.
- Blue LED `PWM.duty_u16` → `ledcSetup(0, 1000, 16)` + `ledcWrite(65535*pct/100)` (identical duty math; 16 bits @ 1 kHz is within the 80 MHz LEDC clock budget).
- MicroPython `i2c.scan()` → address-probe loop 0x08..0x77 excluding 0x77 (`hc_i2c_scan`).
- Parser `bytes` slicing → static 512 B buffer + `memmove` sliding window, identical resync semantics.
- `consecutive_errors` widened to uint16 (Python int was unbounded; reply saturates at 255 exactly like the reference).
- `Serial2.read(buf, n)` confirmed non-blocking (`uartReadBytes(..., timeout=0)` in arduino-esp32 2.0.17) — parser never stalls.

## 9. Build result

```
pio run (env: esp32dev, espressif32 6.10.0, arduino-esp32 2.0.17)
RESULT: SUCCESS
RAM:   6.9%  (22584 / 327680 bytes)
Flash: 22.7% (297121 / 1310720 bytes)
```

## 10. Bench verification checklist (needs the physical rig)

1. `pio run -t upload` + `pio device monitor` → self-test silent, then
   `HAPTIC_SLAVE_READY (UART2 @ 115200 baud, Binary 20B 0x300/0x301, Anti-Error Failsafe Active)` and the console-help line; `>G<` shows all `0.000`.
2. `>S<` → channels 0–2: `60`, 3–4: `61`, others `-`.
3. CONTROL_GUI haptic panel → "Ping ESP32 (UART 115200)": RESP_OK relayed with FW 1.0, token echo, state, uptime.
4. 100 Hz stream: blue LED breathing; `>G<` voltages follow forces, clamped ≤ 2.20 V; stop stream → vents to 0.00 V within ~150 ms, green LED returns.
5. Fault injection (corrupt frames / unplug STM32 TX): 5 errors → red blink + vent; 3 clean frames → recovers to RUNNING.

## 11. Files

- Created: `include/config.h`, `include/haptic_core.h`, `include/tue_protocol.h`, `src/haptic_core.cpp`, `src/tue_protocol.cpp`, this report.
- Rewritten: `src/main.cpp` (was the PlatformIO template stub).
- Unchanged: `platformio.ini`, `src/main.py` (kept as the MicroPython reference).
