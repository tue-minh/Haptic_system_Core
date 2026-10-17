"""Tactile-to-pneumatic haptic synthesis and 100 Hz USB streaming.

Host side of the master/slave teleoperation bridge described in
``TODO_SEND_SENSOR_DATA_THROUGH_USB.md``.  The two 52-taxel jaws are compressed
into one 8-byte payload that maps onto the slave thimble's five pneumatic
valves (A, B, C, D, E), packed into a 20-byte TUE binary frame at CAN ID
``0x300`` and streamed over USB CDC to the STM32 master at 100 Hz.

Three deliberately separable blocks, each unit-testable without hardware
(see ``tests/test_haptic.py``)::

    HapticPayloadSynthesizer  -> 8-byte payload      (feature extraction)
    TueBinaryEncoder          -> 20-byte wire frame  (SOF/CRC/EOF envelope)
    Stm32HapticStreamer       -> 100 Hz serial thread (regulated transmission)

CAN ID ``0x300`` lives in the range the firmware reserves for user custom
communication, which is why the streamer writes the raw frames itself instead
of going through ``tue_usb.TueUsbClient`` (whose ``transact`` refuses reserved
IDs).
"""
from __future__ import annotations

import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import numpy as np

from . import _paths  # noqa: F401  (ensures tue_usb is importable)
from tue_usb import Frame, FrameType, crc8_selftest

try:  # pyserial is a hard requirement; deps.verify() reports it if missing.
    import serial
except Exception:  # pragma: no cover - depends on env
    serial = None

# ---------------------------------------------------------------------------
# Wire / protocol constants (TODO §2.2, §4, §5)
# ---------------------------------------------------------------------------
SOF1 = 0xAA
SOF2 = 0x55
EOF1 = 0x55
EOF2 = 0xAA
FRAME_TYPE = 0x01        # FRAME_TYPE_REQ (host -> MCU)
CAN_ID = 0x00000300      # reserved user-custom range
DLC = 0x08               # payload length

PAYLOAD_LEN = 8
FRAME_LEN = 20           # 2 SOF + 7 header + 8 payload + 1 CRC + 2 EOF

HAPTIC_BAUDRATE = 1_000_000
HAPTIC_RATE_HZ = 100.0
HAPTIC_PERIOD_S = 0.010

FORCE_LSB_N = 0.1        # 1 LSB = 0.1 N on the wire (0..25.5 N)
MAX_FORCE_N = 25.5

N_TAXELS = 52
# 3-zone spatial partition along the finger, Tip -> Base (TODO §3.3)
ZONE_TIP = (0, 16)
ZONE_MID = (16, 36)
ZONE_BASE = (36, 52)

# Status_Flags bitmask (TODO §3.4)
FLAG_LEFT_CONTACT = 0x01
FLAG_RIGHT_CONTACT = 0x02
FLAG_SLIP = 0x04
FLAG_TARE = 0x08
FLAG_NAMES = (
    (FLAG_LEFT_CONTACT, "L-contact"),
    (FLAG_RIGHT_CONTACT, "R-contact"),
    (FLAG_SLIP, "slip"),
    (FLAG_TARE, "tare"),
)

# Physical layout of the 5 pneumatic chambers on the operator's finger, in a
# finger-centric millimetre frame: x across the finger width, y from tip (0)
# to base.  A/B/C run along the underside pad, D/E sit on the two flanks.
VALVE_LAYOUT = (
    ("A", "Tip", 0.0, 0.0),
    ("B", "Mid", 0.0, 11.0),
    ("C", "Base", 0.0, 22.0),
    ("D", "Left flank", -7.0, 11.0),
    ("E", "Right flank", 7.0, 11.0),
)
VALVE_NAMES = tuple(v[0] for v in VALVE_LAYOUT)


def raise_timer_resolution() -> bool:
    """Ask Windows for a 1 ms timer so ``time.sleep`` can honour a 10 ms period.

    The default Windows timer granularity is ~15.6 ms, which caps a naive
    sleep-driven loop at ~64 Hz and would miss the 100 Hz/±0.5 ms requirement.
    No-op (returns ``False``) off Windows.
    """
    if sys.platform != "win32":
        return False
    try:
        import ctypes

        return ctypes.windll.winmm.timeBeginPeriod(1) == 0
    except Exception:
        return False


def restore_timer_resolution() -> None:
    """Release the 1 ms timer request taken by :func:`raise_timer_resolution`."""
    if sys.platform != "win32":
        return
    try:
        import ctypes

        ctypes.windll.winmm.timeEndPeriod(1)
    except Exception:
        pass


def clamp_u8(value: float) -> int:
    """Quantise a force in newtons to the 0.1 N wire LSB, clamped to 0..255."""
    if value is None:
        return 0
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0
    if v != v or v in (float("inf"), float("-inf")):  # NaN / inf
        return 0
    return max(0, min(255, int(round(v * 10.0))))


def _fz_vector(data) -> np.ndarray:
    """Extract a length-52 Fz vector from a snapshot dict, padding/truncating.

    ``taxels_fz`` may be a 52-vector or the (52, 3) taxel array; a missing or
    malformed array yields zeros (TODO §6.3 array-dimension invariant).
    """
    arr = None
    if isinstance(data, dict):
        arr = data.get("taxels_fz")
    elif data is not None:
        arr = data
    if arr is None:
        return np.zeros(N_TAXELS, dtype=float)

    a = np.asarray(arr, dtype=float)
    if a.size == N_TAXELS * 3:
        a = a.reshape(-1, 3)[:, 2]
    else:
        a = a.ravel()
    if a.size < N_TAXELS:
        a = np.concatenate([a, np.zeros(N_TAXELS - a.size)])
    elif a.size > N_TAXELS:
        a = a[:N_TAXELS]
    a = np.nan_to_num(a, nan=0.0, posinf=0.0, neginf=0.0)
    return np.clip(a, 0.0, None)


class HapticPayloadSynthesizer:
    """Compress dual 52-taxel jaws into the 8-byte valve payload (TODO §3).

    Stateless apart from the rolling ``Frame_Counter``.  Each input dict carries
    the contract of TODO §2.1 (``fz``/``fy``/``fx``/``cop_x``/``cop_y``/
    ``taxels_fz``); optional ``t`` (host timestamp) or ``age_s`` drives the
    stale-data guard, which zeroes a side whose last frame is older than
    ``stale_s`` (TODO §6.1).
    """

    def __init__(
        self,
        shear_axis: str = "Y",
        contact_thresh: float = 0.20,
        force_thresh: float = 0.20,
        slip_mu: float = 0.50,
        stale_s: float = 0.10,
    ) -> None:
        axis = str(shear_axis).upper()
        if axis not in ("X", "Y"):
            raise ValueError("shear_axis must be 'X' or 'Y'")
        self.shear_axis = axis
        self.contact_thresh = float(contact_thresh)
        self.force_thresh = float(force_thresh)
        self.slip_mu = float(slip_mu)
        self.stale_s = float(stale_s)
        self._counter = 0

    # -- helpers ------------------------------------------------------------
    @staticmethod
    def _age(data, now: float) -> float:
        if not isinstance(data, dict):
            return 0.0
        if data.get("age_s") is not None:
            return max(0.0, float(data["age_s"]))
        if data.get("t") is not None:
            return max(0.0, now - float(data["t"]))
        return 0.0

    def _side(self, data, now: float):
        """Return ``(fz, shear, fz_vector, fresh)`` for one jaw."""
        if not data:
            return 0.0, 0.0, np.zeros(N_TAXELS), False
        fresh = self._age(data, now) <= self.stale_s
        fz = float(data.get("fz", 0.0) or 0.0)
        key = "fx" if self.shear_axis == "X" else "fy"
        shear = abs(float(data.get(key, 0.0) or 0.0))
        fzvec = _fz_vector(data)
        if not fresh:
            return 0.0, 0.0, np.zeros(N_TAXELS), False
        return fz, shear, fzvec, True

    def _slip(self, data, fresh: bool) -> bool:
        if not fresh or not isinstance(data, dict):
            return False
        fz = float(data.get("fz", 0.0) or 0.0)
        if fz <= self.force_thresh:
            return False
        key = "fx" if self.shear_axis == "X" else "fy"
        shear = abs(float(data.get(key, 0.0) or 0.0))
        return (shear / fz) > self.slip_mu

    # -- primary API --------------------------------------------------------
    def synthesize(
        self,
        left_data,
        right_data,
        slip_alert: bool = False,
        tare_active: bool = False,
        now: Optional[float] = None,
    ) -> bytes:
        """Return exactly 8 payload bytes (``struct.pack("BBBBBBBB", ...)``)."""
        if now is None:
            now = time.perf_counter()

        fz_l, shear_l, tax_l, fresh_l = self._side(left_data, now)
        fz_r, shear_r, tax_r, fresh_r = self._side(right_data, now)

        byte_fz_l = clamp_u8(fz_l)
        byte_fz_r = clamp_u8(fz_r)
        shear_avg = (shear_l + shear_r) / 2.0
        byte_shear = clamp_u8(shear_avg)

        combined = tax_l + tax_r
        s_total = float(combined.sum())
        if s_total > self.contact_thresh:
            s1 = float(combined[ZONE_TIP[0]:ZONE_TIP[1]].sum())
            s2 = float(combined[ZONE_MID[0]:ZONE_MID[1]].sum())
            w_a = min(255, int(round(255.0 * s1 / s_total)))
            w_b = min(255, int(round(255.0 * s2 / s_total)))
            w_c = max(0, 255 - (w_a + w_b))
        else:
            w_a = w_b = w_c = 0

        flags = 0
        if fresh_l and fz_l > self.force_thresh:
            flags |= FLAG_LEFT_CONTACT
        if fresh_r and fz_r > self.force_thresh:
            flags |= FLAG_RIGHT_CONTACT
        if slip_alert or self._slip(left_data, fresh_l) or self._slip(right_data, fresh_r):
            flags |= FLAG_SLIP
        if tare_active:
            flags |= FLAG_TARE

        payload = struct.pack(
            "BBBBBBBB", byte_fz_l, byte_fz_r, byte_shear,
            w_a, w_b, w_c, flags, self._counter,
        )
        self._counter = (self._counter + 1) & 0xFF
        return payload


class TueBinaryEncoder:
    """Assemble the 20-byte TUE binary envelope around an 8-byte payload.

    The envelope is the shared ``tue_usb`` frame used everywhere else on the
    link - identical SOF/header/CRC/EOF layout, only the CAN ID differs
    (``0x300``).  It is built with ``tue_usb.Frame`` so the CRC-8 stays
    byte-for-byte the firmware's; this class just owns the rolling header
    ``SEQ`` counter.
    """

    SOF1 = SOF1
    SOF2 = SOF2
    EOF1 = EOF1
    EOF2 = EOF2
    FRAME_TYPE = FRAME_TYPE
    CAN_ID = CAN_ID
    DLC = DLC

    def __init__(self) -> None:
        self._seq = 0

    @property
    def seq(self) -> int:
        return self._seq

    def encode(self, payload: bytes) -> bytes:
        payload = bytes(payload)
        if len(payload) != self.DLC:
            raise ValueError(f"payload must be exactly {self.DLC} bytes, got {len(payload)}")

        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        frame = Frame(frame_type=FrameType.REQ, can_id=self.CAN_ID, payload=payload, seq=seq)
        return frame.to_bytes()


# ---------------------------------------------------------------------------
# 5-valve view model (what the GUI visualises)
# ---------------------------------------------------------------------------
@dataclass
class HapticValve:
    """One pneumatic chamber as driven by the latest payload."""

    name: str
    label: str
    x: float
    y: float
    force: float = 0.0     # rendered force in newtons
    weight: float = 0.0    # 0..1 activation weight / normalised byte
    raw: int = 0           # the payload byte driving this valve
    t: float = 0.0         # host time of the frame that last drove it
    age: float = 0.0       # seconds since that frame (data freshness)


@dataclass
class HapticSnapshot:
    """Thread-safe copy of the streamer's latest state for the GUI."""

    state: str = "idle"
    connected: bool = False
    error: str = ""
    seq: int = 0
    t: float = 0.0
    age: float = 0.0
    flags: int = 0
    frame: bytes = b""
    valves: Tuple[HapticValve, ...] = field(default_factory=tuple)
    sent: int = 0
    errors: int = 0
    rate_hz: float = 0.0

    @property
    def flag_names(self) -> List[str]:
        return [name for bit, name in FLAG_NAMES if self.flags & bit]

    @property
    def frame_hex(self) -> str:
        return " ".join(f"{b:02X}" for b in self.frame) if self.frame else "---"

    @classmethod
    def from_payload(
        cls,
        payload: bytes,
        t: float,
        age: float = 0.0,
        *,
        seq: int = 0,
        frame: bytes = b"",
        state: str = "streaming",
        connected: bool = True,
        error: str = "",
        sent: int = 0,
        errors: int = 0,
        rate_hz: float = 0.0,
    ) -> "HapticSnapshot":
        if len(payload) < PAYLOAD_LEN:
            payload = bytes(payload) + b"\x00" * (PAYLOAD_LEN - len(payload))

        fz_l = payload[0] * FORCE_LSB_N
        fz_r = payload[1] * FORCE_LSB_N
        shear = payload[2] * FORCE_LSB_N
        w_a, w_b, w_c = payload[3], payload[4], payload[5]
        flags = payload[6]

        forces = {
            "A": shear * (w_a / 255.0),
            "B": shear * (w_b / 255.0),
            "C": shear * (w_c / 255.0),
            "D": fz_l,
            "E": fz_r,
        }
        weights = {
            "A": w_a / 255.0, "B": w_b / 255.0, "C": w_c / 255.0,
            "D": payload[0] / 255.0, "E": payload[1] / 255.0,
        }
        raws = {"A": w_a, "B": w_b, "C": w_c, "D": payload[0], "E": payload[1]}

        valves = tuple(
            HapticValve(
                name=name, label=label, x=x, y=y,
                force=forces[name], weight=weights[name], raw=raws[name],
                t=t, age=age,
            )
            for name, label, x, y in VALVE_LAYOUT
        )
        return cls(
            state=state, connected=connected, error=error, seq=seq, t=t, age=age,
            flags=flags, frame=bytes(frame), valves=valves,
            sent=sent, errors=errors, rate_hz=rate_hz,
        )


# ---------------------------------------------------------------------------
# 100 Hz transmission thread
# ---------------------------------------------------------------------------
class Stm32HapticStreamer(threading.Thread):
    """Regulated 100 Hz serial writer for the synthesised haptic frames.

    The GUI pushes fresh sensor snapshots with :meth:`update_sensors`; the
    thread re-synthesises and writes a frame every ``period`` seconds so the
    USB CDC link carries a deterministic 100 Hz stream regardless of how fast
    the sensors or the GUI tick (TODO §5 Step 3).
    """

    MAX_CONSECUTIVE_ERRORS = 50

    def __init__(
        self,
        port: str,
        baudrate: int = HAPTIC_BAUDRATE,
        log: Optional[Callable[[str], None]] = None,
        period: float = HAPTIC_PERIOD_S,
        synthesizer: Optional[HapticPayloadSynthesizer] = None,
        serial_timeout: float = 0.05,
        clock: Optional[Callable[[], float]] = None,
        shear_axis: str = "Y",
    ) -> None:
        super().__init__(daemon=True, name=f"Haptic-{port}")
        self.port = port
        self.baudrate = int(baudrate)
        self._period = max(0.001, float(period))
        self._timeout = float(serial_timeout)
        self._log_fn = log or (lambda _m: None)
        self._clock = clock or time.perf_counter

        self.synthesizer = synthesizer or HapticPayloadSynthesizer(shear_axis=shear_axis)
        self.encoder = TueBinaryEncoder()

        self._lock = threading.Lock()
        self._left = None
        self._right = None
        self._left_t = 0.0
        self._right_t = 0.0
        self._slip = False
        self._tare = False

        self._stop = threading.Event()
        self._ser = None
        self._timer_raised = False
        self._state = "idle"
        self._error = ""
        self._last_payload: Optional[bytes] = None
        self._last_frame: bytes = b""
        self._last_t = 0.0
        self._sent = 0
        self._errors = 0
        self._consecutive = 0
        self._rate_hz = 0.0

    # -- lifecycle ----------------------------------------------------------
    def log(self, msg: str) -> None:
        self._log_fn(f"[haptic {self.port}] {msg}")

    def open(self) -> bool:
        """Open the serial port (idempotent). Returns ``True`` on success."""
        if self._ser is not None:
            return True
        if serial is None:
            self._set_error("pyserial is not installed")
            return False
        try:
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self._timeout,
                write_timeout=self._timeout,
            )
        except Exception as exc:
            self._set_error(f"open failed: {exc}")
            return False
        self._state = "streaming"
        self._error = ""
        self._timer_raised = raise_timer_resolution()
        self.log(f"open @ {self.baudrate} baud -> streaming {HAPTIC_RATE_HZ:.0f} Hz")
        return True

    def close(self) -> None:
        """Signal stop and release the serial handle."""
        self._stop.set()
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if self._timer_raised:
            restore_timer_resolution()
            self._timer_raised = False
        if self._state != "error":
            self._state = "stopped"

    def shutdown(self) -> None:
        """Ask the thread to finish (join it to wait for the port to close)."""
        self._stop.set()

    def is_open(self) -> bool:
        return self._ser is not None

    # -- input --------------------------------------------------------------
    def update_sensors(
        self,
        left_data,
        right_data,
        slip_alert: bool = False,
        tare_active: bool = False,
    ) -> None:
        """Thread-safe hand-off of the newest sensor snapshots."""
        now = self._clock()
        with self._lock:
            self._left = left_data
            self._right = right_data
            self._left_t = self._stamp(left_data, now)
            self._right_t = self._stamp(right_data, now)
            self._slip = bool(slip_alert)
            self._tare = bool(tare_active)

    def _stamp(self, data, now: float) -> float:
        if isinstance(data, dict) and data.get("t") is not None:
            return float(data["t"])
        return now

    # -- status -------------------------------------------------------------
    def snapshot(self) -> HapticSnapshot:
        with self._lock:
            payload = self._last_payload
            frame = self._last_frame
            t = self._last_t
            seq = self.encoder.seq
            state = self._state
            error = self._error
            sent = self._sent
            errors = self._errors
            rate = self._rate_hz
        if payload is None:
            return HapticSnapshot(state=state, connected=self.is_open(), error=error,
                                  sent=sent, errors=errors, rate_hz=rate)
        age = max(0.0, self._clock() - t)
        return HapticSnapshot.from_payload(
            payload, t, age, seq=seq, frame=frame, state=state,
            connected=self.is_open(), error=error, sent=sent, errors=errors,
            rate_hz=rate,
        )

    @property
    def state(self) -> str:
        return self._state

    @property
    def error(self) -> str:
        return self._error

    # -- thread body --------------------------------------------------------
    def run(self) -> None:
        if not self.open():
            return

        next_t = time.perf_counter()
        rate_t0 = time.perf_counter()
        rate_cnt = 0
        while not self._stop.is_set():
            now = time.perf_counter()
            if now < next_t:
                self._stop.wait(next_t - now)
                if self._stop.is_set():
                    break
                now = time.perf_counter()
            next_t += self._period
            if next_t < now:
                next_t = now + self._period

            self._emit_once()

            rate_cnt += 1
            if now - rate_t0 >= 1.0:
                self._rate_hz = rate_cnt / max(now - rate_t0, 1e-9)
                rate_t0, rate_cnt = now, 0

        self.close()
        self.log("stopped")

    def _emit_once(self) -> None:
        clock_now = self._clock()
        with self._lock:
            left_age = max(0.0, clock_now - self._left_t) if self._left is not None else 0.0
            right_age = max(0.0, clock_now - self._right_t) if self._right is not None else 0.0
            left = _with_age(self._left, left_age)
            right = _with_age(self._right, right_age)
            slip = self._slip
            tare = self._tare
            synth = self.synthesizer

        payload = synth.synthesize(left, right, slip_alert=slip, tare_active=tare, now=clock_now)
        frame = self.encoder.encode(payload)

        ser = self._ser
        if ser is None:
            return
        try:
            ser.write(frame)
        except Exception as exc:
            self._on_write_error(exc)
            return

        self._consecutive = 0
        with self._lock:
            self._sent += 1
            self._last_payload = payload
            self._last_frame = frame
            self._last_t = clock_now

    def _on_write_error(self, exc: Exception) -> None:
        with self._lock:
            self._errors += 1
            self._consecutive += 1
            count = self._consecutive
        self.log(f"write failed ({count}): {exc}")
        if count >= self.MAX_CONSECUTIVE_ERRORS:
            self._set_error(f"lost contact: {exc}")
            self._stop.set()

    def _set_error(self, msg: str) -> None:
        with self._lock:
            self._state = "error"
            self._error = msg
        self.log(f"ERROR: {msg}")


def _with_age(data, age: float):
    """Return a shallow copy of a snapshot dict carrying an explicit age."""
    if not isinstance(data, dict):
        return data
    out = dict(data)
    out["age_s"] = float(age)
    return out


def self_test() -> bool:
    """Verify the CRC reference vector and the three acceptance payloads."""
    if not crc8_selftest():
        return False

    synth = HapticPayloadSynthesizer()
    zero_l = {"fz": 0.0, "fy": 0.0, "taxels_fz": [0.0] * N_TAXELS}
    zero_r = dict(zero_l)
    if synth.synthesize(zero_l, zero_r, now=0.0)[:7] != b"\x00" * 7:
        return False

    pinch_l = {"fz": 5.0, "fy": 0.0, "taxels_fz": [0.0] * N_TAXELS}
    pinch_r = {"fz": 3.0, "fy": 0.0, "taxels_fz": [0.0] * N_TAXELS}
    if synth.synthesize(pinch_l, pinch_r, now=0.0)[:7] != bytes([0x32, 0x1E, 0, 0, 0, 0, 0x03]):
        return False

    tax = [0.0] * N_TAXELS
    tax[ZONE_TIP[0]:ZONE_TIP[1]] = [1.0] * (ZONE_TIP[1] - ZONE_TIP[0])
    shear = {"fz": 2.0, "fy": 1.0, "taxels_fz": tax}
    if synth.synthesize(shear, shear, now=0.0)[:7] != bytes([0x14, 0x14, 0x0A, 0xFF, 0, 0, 0x03]):
        return False

    enc = TueBinaryEncoder()
    frame = enc.encode(synth.synthesize(zero_l, zero_r, now=0.0))
    if len(frame) != FRAME_LEN or frame[0] != SOF1 or frame[-1] != EOF2:
        return False
    return True


if __name__ == "__main__":
    print("haptic self-test:", "PASS" if self_test() else "FAIL")
