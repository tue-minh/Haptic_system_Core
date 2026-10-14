"""
paxini_core.py
==============
Protocol, streaming reader, metrics and CSV recorder for PaXini GEN3 tactile
sensors (e.g. PX6AX-GEN3-DP-S2015-Elite) connected through the
High-Speed Communication Board (USB, 921600 baud).

Based on the vendor reference code (Hand_UI.py / Read_Single_Sensor_Hand.py)
and "PaXini PX-6AX GEN3 Communication Board Communication Protocol V1.0.5".

This module has NO Qt dependency; the GUI lives in paxini_multi_gui.py.

Threading model (for performance):
  * one SensorWorker thread per COM port: reads bytes, parses frames, computes
    metrics with numpy and appends them to a lock-protected ring buffer;
  * one CsvRecorder thread per recorded module: batches rows and writes them
    with np.savetxt, so file I/O never blocks the serial reader;
  * the GUI only *polls* the ring buffers at a fixed frame rate (no per-frame
    Qt signals), which keeps the UI responsive at high sensor rates.
"""
from __future__ import annotations

import math
import os
import queue
import threading
import time
from typing import Callable, List, Optional

import numpy as np
import serial
import serial.tools.list_ports

# --------------------------------------------------------------------------
# Protocol constants (High-Speed Communication Board)
# --------------------------------------------------------------------------
BAUDRATE = 921600
REQ_HEAD = b"\x55\xAA"    # host -> board
RESP_HEAD = b"\xAA\x55"   # board -> host (request/response mode)
PUSH_HEAD = b"\xAA\x56"   # board -> host (automatic transmission mode)
FUNC_READ = 0x03
FUNC_WRITE = 0x10

REG_VERSION = 0x0000      # 15/16 bytes ASCII
REG_COMBINATION = 0x0010  # 0x0010-0x0013 connected module bitmap (4 bytes)
REG_DATA_TYPE = 0x0016    # bit0 resultant force, bit1 force array (default 0x03)
REG_AUTO_PUSH = 0x0017    # 1 = enable auto transmission, 0 = disable
REG_POINTS = 0x0030       # 0x0030-0x0067 taxel count per module (uint16 x 28)

FORCE_LSB_N = 0.1         # 1 LSB = 0.1 N

MODULE_NAMES = [
    "Thumb-Proximal", "Thumb-Middle", "Thumb-Distal", "Thumb-Nail",
    "Index-Proximal", "Index-Middle", "Index-Distal", "Index-Nail",
    "Middle-Proximal", "Middle-Middle", "Middle-Distal", "Middle-Nail",
    "Ring-Proximal", "Ring-Middle", "Ring-Distal", "Ring-Nail",
    "Little-Proximal", "Little-Middle", "Little-Distal", "Little-Nail",
    "Palm1", "Palm2", "Palm3", "Palm4", "Palm5", "Palm6", "Palm7", "Palm8",
]

# Taxel coordinates (mm) of PX6AX-GEN3-DP-S2015-Elite (52 taxels), taken from
# "ã€06ã€‘Coordinates of Fore Signals in Array / Fingertip 15mm ... S2015-Elite.xlsx"
# Columns: X, Y, Z. Row k corresponds to taxel #k+1 (= k-th force triplet in the frame).
S2015_ELITE_XYZ = np.array([
    [-7.45866948, -0.80442889, 1.9377763], [-7.24032101, 1.9370259, 1.93780601],
    [-7.00988064, 5.22921165, 1.93775712], [-6.82610763, 7.97333403, 1.93769891],
    [-6.53146658, 11.26026387, 1.93782951], [-6.00006622, -1.35478789, 4.53417757],
    [-6.00372647, 1.50179184, 4.67021242], [-5.94055243, 4.93336235, 4.69448457],
    [-5.81660022, 7.79060929, 4.65440797], [-6.07106495, 13.96973406, 1.93782184],
    [-5.51333141, 11.20680746, 4.536848], [-5.0017979, 14.00765355, 4.28734202],
    [-5.09707632, 16.57086028, 1.93780027], [-4.40866763, 16.2194301, 3.70766158],
    [-3.1037709, -1.55570508, 5.41014857], [-3.15461212, 1.5038129, 5.90611644],
    [-3.20348468, 5.19993, 6.32908896], [-3.21601164, 8.3044369, 6.53483487],
    [-3.1409429, 11.42030336, 6.47498956], [-2.83527189, 15.06167934, 5.87056449],
    [-2.42756984, 17.70908967, 4.36265159], [-2.71496388, 18.81872693, 1.93777956],
    [0.0, -1.55808447, 5.4116888], [0.0, 1.60830268, 5.95092329],
    [0.0, 5.43294958, 6.43279476], [0.0, 8.63424733, 6.69381112],
    [0.0, 12.48579856, 6.62785389], [0.0, 15.63993516, 6.05667345],
    [0.0, 18.02972832, 4.50323706], [0.0, 19.32616275, 1.93777338],
    [3.15461212, 1.5038129, 5.90611644], [3.1409429, 11.42030336, 6.47498956],
    [2.83527189, 15.06167934, 5.87056449], [2.42756984, 17.70908967, 4.36265159],
    [3.1037709, -1.55570508, 5.41014857], [3.20348468, 5.19993, 6.32908896],
    [3.21601164, 8.3044369, 6.53483487], [2.71496388, 18.81872693, 1.93777956],
    [4.40866763, 16.2194301, 3.70766158], [5.0017979, 14.00765355, 4.28734202],
    [5.09707632, 16.57086028, 1.93780027], [6.00006622, -1.35478789, 4.53417757],
    [6.00372647, 1.50179184, 4.67021242], [5.94055243, 4.93336235, 4.69448457],
    [5.81660022, 7.79060929, 4.65440797], [5.51333141, 11.20680746, 4.536848],
    [6.53146658, 11.26026387, 1.93782951], [6.07106495, 13.96973406, 1.93782184],
    [7.24032101, 1.9370259, 1.93780601], [7.00988064, 5.22921165, 1.93775712],
    [6.82610763, 7.97333403, 1.93769891], [7.45866948, -0.80442889, 1.9377763],
], dtype=np.float64)

# Known taxel layouts, keyed by taxel count
KNOWN_LAYOUTS = {52: ("S2015-Elite", S2015_ELITE_XYZ)}

# Columns stored in the ring buffer (one row per received frame and module)
METRIC_COLUMNS = [
    "t",          # 0  host time [s] since app start (shared across all ports)
    "Fx",         # 1  resultant Fx [N]
    "Fy",         # 2  resultant Fy [N]
    "Fz",         # 3  resultant Fz [N] (normal)
    "F_mag",      # 4  |F| [N]
    "F_shear",    # 5  tangential magnitude sqrt(Fx^2+Fy^2) [N]
    "mu",         # 6  Ft/Fz (tangential/normal ratio, friction utilisation)
    "CoP_x",      # 7  centre of pressure x [mm] (Fz-weighted taxel centroid)
    "CoP_y",      # 8  centre of pressure y [mm]
    "shear_dir",  # 9  direction of tangential force atan2(Fy,Fx) [deg]
    "tilt",       # 10 tilt of force vector from surface normal atan2(Ft,Fz) [deg]
    "peak_idx",   # 11 taxel # (1-based) with the highest Fz = contact point
    "peak_Fz",    # 12 Fz at that taxel [N]
    "contact_n",  # 13 number of taxels above threshold (contact size)
]
COL = {name: i for i, name in enumerate(METRIC_COLUMNS)}

APP_T0 = time.perf_counter()


def now() -> float:
    """Monotonic host time in seconds since the application started."""
    return time.perf_counter() - APP_T0


class Config:
    """Runtime-tunable thresholds (read by reader threads, set by GUI)."""
    taxel_threshold_n = 0.1   # a taxel is "in contact" if its Fz >= this
    force_threshold_n = 0.2   # below this |F|, direction/CoP are reported as NaN


CONFIG = Config()


# --------------------------------------------------------------------------
# Low-level helpers
# --------------------------------------------------------------------------
def calc_lrc(data: bytes) -> int:
    """LRC: 8-bit sum -> two's complement."""
    return (-sum(data)) & 0xFF


def build_frame(func: int, reg: int, length: int, data: bytes = b"") -> bytes:
    body = (REQ_HEAD + b"\x00" + bytes([func]) + reg.to_bytes(2, "little")
            + length.to_bytes(2, "little") + data)
    return body + bytes([calc_lrc(body)])


def list_ports():
    return list(serial.tools.list_ports.comports())


def transact(ser: serial.Serial, frame: bytes, head: bytes = RESP_HEAD,
             timeout: float = 2.0, log: Optional[Callable[[str], None]] = None) -> Optional[bytes]:
    """Send a request and wait for one complete, LRC-valid response frame."""
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.1)  # Give board time to process and reply
    buf = bytearray()
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
        while True:
            i = buf.find(head)
            if i < 0:
                break
            if head == RESP_HEAD:
                if len(buf) - i < 8:
                    break
                total = 9 + int.from_bytes(buf[i + 6:i + 8], "little")
            else:
                if len(buf) - i < 5:
                    break
                total = 6 + int.from_bytes(buf[i + 3:i + 5], "little")
            if len(buf) - i < total:
                break
            fr = bytes(buf[i:i + total])
            if calc_lrc(fr[:-1]) == fr[-1]:
                return fr
            del buf[:i + 2]          # bad LRC, resync
    if log:
        hex_dump = buf.hex()
        log(f"Timeout waiting for response. Buffer ({len(buf)} bytes): {hex_dump[:200]}..." if len(hex_dump) > 200 else f"Timeout waiting for response. Buffer: {hex_dump}")
    return None


def _points_offset(module_idx: int) -> int:
    """Byte offset inside the 0x0030..0x0067 block for a module's taxel count.
    Fingers are in order; palms are stored reversed (0x0058 = palm8 ... 0x0066 = palm1)."""
    if module_idx < 20:
        return 2 * module_idx
    palm_k = module_idx - 19          # 1..8
    return (0x58 - 0x30) + 2 * (8 - palm_k)


# --------------------------------------------------------------------------
# Ring buffer (double-write so the latest window is always contiguous)
# --------------------------------------------------------------------------
class RingBuffer:
    def __init__(self, capacity: int, ncols: int):
        self.cap = capacity
        self.buf = np.full((2 * capacity, ncols), np.nan)
        self.i = 0
        self.n = 0
        self.lock = threading.Lock()

    def append(self, row):
        with self.lock:
            self.buf[self.i] = row
            self.buf[self.i + self.cap] = row
            self.i = (self.i + 1) % self.cap
            if self.n < self.cap:
                self.n += 1

    def window(self, t_from: float, cols: List[int]) -> np.ndarray:
        """Copy of rows with t >= t_from, only the requested columns."""
        with self.lock:
            if self.n == 0:
                return np.empty((0, len(cols)))
            end = self.i + self.cap
            view = self.buf[end - self.n:end]
            s = int(np.searchsorted(view[:, 0], t_from))
            return view[s:, cols].copy()

    def clear(self):
        with self.lock:
            self.i = 0
            self.n = 0


# --------------------------------------------------------------------------
# CSV recorder (background writer thread)
# --------------------------------------------------------------------------
class CsvRecorder:
    def __init__(self, path: str, n_points: int, t0: float, meta: str = ""):
        self.path = path
        self.n_points = n_points
        self.t0 = t0
        self.rows = 0
        self._q: "queue.SimpleQueue" = queue.SimpleQueue()
        self._f = open(path, "w", newline="", encoding="utf-8")
        cols = ["t_rel_s", "epoch_s"] + [
            "Fx_N", "Fy_N", "Fz_N", "F_mag_N", "F_shear_N", "mu_Ft_over_Fz",
            "CoP_x_mm", "CoP_y_mm", "shear_dir_deg", "tilt_deg",
            "peak_taxel", "peak_Fz_N", "contact_taxels"]
        for k in range(1, n_points + 1):
            cols += [f"t{k}_Fx", f"t{k}_Fy", f"t{k}_Fz"]
        self._ncols = len(cols)
        if meta:
            self._f.write(f"# {meta}\n")
        self._f.write(",".join(cols) + "\n")
        self._fmt = ["%.6f", "%.6f"] + ["%.3f"] * 13 + ["%.1f"] * (3 * n_points)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def push(self, row, epoch: float, taxels: np.ndarray):
        self._q.put((row, epoch, taxels))

    def _run(self):
        done = False
        while not done:
            try:
                item = self._q.get(timeout=0.25)
            except queue.Empty:
                continue
            items = []
            while True:
                if item is None:
                    done = True
                    break
                items.append(item)
                if len(items) >= 5000:
                    break
                try:
                    item = self._q.get_nowait()
                except queue.Empty:
                    break
            if not items:
                continue
            arr = np.full((len(items), self._ncols), np.nan)
            for k, (row, ep, tax) in enumerate(items):
                arr[k, 0] = row[0] - self.t0
                arr[k, 1] = ep
                arr[k, 2:15] = row[1:]
                if tax is not None and tax.size == 3 * self.n_points:
                    arr[k, 15:] = tax.ravel()
            np.savetxt(self._f, arr, fmt=self._fmt, delimiter=",")
            self.rows += len(items)
        self._f.flush()

    def stop(self) -> int:
        self._q.put(None)
        self._thread.join(timeout=10)
        try:
            self._f.close()
        except Exception:
            pass
        return self.rows


# --------------------------------------------------------------------------
# USB Converter Board Protocol and Sensor Logic
# --------------------------------------------------------------------------
def calc_usb_lrc(data: bytes) -> int:
    lrc = 0
    for byte in data:
        lrc = (lrc + byte) & 0xFF
    return ((~lrc) + 1) & 0xFF

def build_usb_frame(hex_str: str) -> bytes:
    frame = bytes.fromhex(hex_str)
    return frame + bytes([calc_usb_lrc(frame)])

def transact_usb(ser: serial.Serial, frame: bytes, min_resp_len: int, timeout: float = 0.5) -> bytes:
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.01)
    response = bytearray()
    start = time.perf_counter()
    while time.perf_counter() - start < timeout:
        if ser.in_waiting:
            response += ser.read(ser.in_waiting)
            idx = response.find(b"\xAA\x55")
            if idx >= 0 and len(response) - idx >= min_resp_len:
                return bytes(response[idx : idx + min_resp_len])
        time.sleep(0.001)
    return bytes(response)


# --------------------------------------------------------------------------
# Device address auto-detection
# --------------------------------------------------------------------------
# Every board answers only on its own bus address, which is stored in the
# board's EEPROM and can be anything in 0x01..0x08.  Rather than making the
# caller know it, we send the calibration request to every candidate address on
# connect and keep the first one that replies.  A board that is still booting
# will miss the first pass, so the sweep is repeated.
PROBE_ADDRESSES = (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08)
PROBE_PASSES = 2
PROBE_TIMEOUT_S = 0.25
PROBE_MIN_RESPONSE = 15


def build_calibration_frame(addr: int) -> bytes:
    """Calibration request addressed to ``addr`` (byte 5 of the frame)."""
    return build_usb_frame(
        f"55 AA 0A 00 {int(addr) & 0xFF:02X} 00 79 03 00 00 00 01 00 01")


def probe_device_address(ser, log: Optional[Callable[[str], None]] = None,
                         preferred: Optional[int] = None,
                         addresses=PROBE_ADDRESSES, passes: int = PROBE_PASSES,
                         timeout: float = PROBE_TIMEOUT_S) -> Optional[int]:
    """Return the bus address the sensor answers on, or ``None`` if silent.

    ``preferred`` is tried first (useful when an address is known to be
    correct), but the remaining candidates are still swept, so a stale
    preference cannot stop a working board from being found.
    """
    order = [int(a) for a in addresses]
    if preferred:
        if preferred in order:
            order.remove(int(preferred))
        order.insert(0, int(preferred))

    for sweep in range(1, max(1, int(passes)) + 1):
        for addr in order:
            resp = transact_usb(ser, build_calibration_frame(addr),
                                PROBE_MIN_RESPONSE, timeout)
            if resp and resp.startswith(RESP_HEAD):
                if log:
                    log(f"device address {addr:02X} answered (sweep {sweep})")
                return addr
        if log:
            tried = "-".join(f"{a:02X}" for a in order)
            log(f"no reply from addresses {tried} (sweep {sweep}/{passes})")
    return None


class UsbSensorWorker(threading.Thread):
    def __init__(self, port: str, log: Callable[[str], None], device_addr: int = 0,
                 model: str = "S2015_elite", baudrate: int = BAUDRATE):
        super().__init__(daemon=True, name=f"PaXiniUSB-{port}")
        self.port = port
        self._log = log
        # 0 = auto-detect the bus address on connect (see probe_device_address);
        # 0x01..0x08 pins the address but is still tried first if detection runs.
        self.device_addr = int(device_addr)
        self.baudrate = int(baudrate)
        self._stop_evt = threading.Event()
        self.state = "connecting"
        self.error = ""
        self.frames = 0
        self.rate_hz = 0.0
        self.timeouts = 0        # missing/partial responses (frame-drop counter)
        
        # Geometry
        self.model = model
        self.n_points = 52 # S2015_elite
        self.xy = S2015_ELITE_XYZ[:, :2].copy()
        self.ring = RingBuffer(65536, len(METRIC_COLUMNS))
        self.latest_row = None
        self.latest_taxels = None
        self.recorder: Optional[CsvRecorder] = None
        
        self.tare_res = np.zeros(3)
        self.tare_tax = np.zeros((self.n_points, 3), np.float32)
        self._tare_left = 0

    def log(self, msg: str):
        self._log(f"[{self.port}] {msg}")

    def stop(self):
        self._stop_evt.set()

    def request_tare(self, n_frames: int = 50):
        self._tare_acc_res = np.zeros(3)
        self._tare_acc_tax = np.zeros((self.n_points, 3))
        self._tare_n = n_frames
        self._tare_left = n_frames

    def reset_tare(self):
        self._tare_left = 0
        self.tare_res = np.zeros(3)
        self.tare_tax = np.zeros((self.n_points, 3), np.float32)

    def run(self):
        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1,
                inter_byte_timeout=0.0005,
                xonxoff=False,
                rtscts=False
            )
            time.sleep(0.5)
            self.log(f"Port opened. Probing for device address...")
        except Exception as e:
            self.state, self.error = "error", str(e)
            self.log(f"Open failed: {e}")
            return

        try:
            self.state = "connecting"
            lo, hi = PROBE_ADDRESSES[0], PROBE_ADDRESSES[-1]
            self.log(f"detecting device address ({lo:02X}-{hi:02X})...")
            found_addr = probe_device_address(
                ser, self.log,
                preferred=self.device_addr or None,
                timeout=PROBE_TIMEOUT_S,
            )

            if found_addr is None:
                self.state = "error"
                self.error = (f"no sensor answered on addresses "
                              f"{lo:02X}-{hi:02X}")
                self.log(f"ERROR: {self.error}")
                return

            self.device_addr = found_addr
            self.state = "streaming"
            self._stream(ser)
        except Exception as e:
            self.state, self.error = "error", str(e)
            self.log(f"Error: {e}")
        finally:
            try:
                ser.close()
            except Exception:
                pass
            if self.state != "error":
                self.state = "stopped"
            self.log("Port closed")

    def _stream(self, ser: serial.Serial):
        # build request frames
        res_hex = f"55 AA 09 00 {self.device_addr:02X} 00 FB F0 03 00 00 03 00"
        res_req = build_usb_frame(res_hex)
        
        dist_len = self.n_points * 3
        dist_hex = f"55 AA 09 00 {self.device_addr:02X} 00 FB 0E 04 00 00 {dist_len & 0xFF:02X} {(dist_len >> 8) & 0xFF:02X}"
        dist_req = build_usb_frame(dist_hex)

        rate_t0 = time.perf_counter()
        rate_cnt = 0

        while not self._stop_evt.is_set():
            t = now()
            ep = time.time()
            
            # Read resultant
            r_resp = transact_usb(ser, res_req, 14 + 3 + 1, 0.1)
            if not r_resp or len(r_resp) < 17:
                self.timeouts += 1
                continue
                
            res_data = r_resp[-4:-1]
            fx_raw, fy_raw, fz_raw = res_data[0], res_data[1], res_data[2]
            fx = (fx_raw if fx_raw <= 127 else fx_raw - 256) * FORCE_LSB_N
            fy = (fy_raw if fy_raw <= 127 else fy_raw - 256) * FORCE_LSB_N
            fz = fz_raw * FORCE_LSB_N
            
            # Read distributed
            d_resp = transact_usb(ser, dist_req, 14 + dist_len + 1, 0.1)
            if not d_resp or len(d_resp) < 14 + dist_len:
                self.timeouts += 1
                continue
                
            dist_data = d_resp[-dist_len - 1 : -1]
            tax_raw = np.frombuffer(dist_data, dtype=np.uint8)
            tax = np.empty((self.n_points, 3), np.float32)
            tax[:, :2] = tax_raw.view(np.int8).reshape(self.n_points, 3)[:, :2]
            tax[:, 2] = tax_raw.reshape(self.n_points, 3)[:, 2]
            tax *= FORCE_LSB_N

            if self._tare_left > 0:
                self._tare_acc_res += (fx, fy, fz)
                self._tare_acc_tax += tax
                self._tare_left -= 1
                if self._tare_left == 0:
                    self.tare_res = self._tare_acc_res / self._tare_n
                    self.tare_tax = (self._tare_acc_tax / self._tare_n).astype(np.float32)

            fx -= self.tare_res[0]
            fy -= self.tare_res[1]
            fz -= self.tare_res[2]
            tax -= self.tare_tax

            # Metrics
            f_shear = math.hypot(fx, fy)
            f_mag = math.sqrt(f_shear * f_shear + fz * fz)
            thr_t = CONFIG.taxel_threshold_n
            thr_f = CONFIG.force_threshold_n
            nan = math.nan

            fz_t = tax[:, 2]
            w = np.where(fz_t >= thr_t, fz_t, 0.0)
            w_sum = float(w.sum())
            contact_n = int(np.count_nonzero(w))
            if w_sum > 0.0 and contact_n:
                cop = (w @ self.xy) / w_sum
                cop_x, cop_y = float(cop[0]), float(cop[1])
            else:
                cop_x = cop_y = nan

            pk = int(np.argmax(fz_t))
            peak_fz = float(fz_t[pk])
            peak_idx = float(pk + 1) if peak_fz >= thr_t else nan

            if f_mag >= thr_f:
                shear_dir = math.degrees(math.atan2(fy, fx))
                tilt = math.degrees(math.atan2(f_shear, fz))
                mu = f_shear / fz if fz >= thr_f else nan
            else:
                shear_dir = tilt = mu = nan

            row = (t, fx, fy, fz, f_mag, f_shear, mu, cop_x, cop_y,
                   shear_dir, tilt, peak_idx, peak_fz, float(contact_n))
            
            self.ring.append(row)
            self.latest_row = row
            self.latest_taxels = tax
            if self.recorder:
                self.recorder.push(row, ep, tax)
                
            self.frames += 1
            rate_cnt += 1
            pc = time.perf_counter()
            if pc - rate_t0 >= 1.0:
                self.rate_hz = rate_cnt / (pc - rate_t0)
                rate_t0, rate_cnt = pc, 0

