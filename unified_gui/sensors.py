"""Dual PaXini tactile sensor management (Phase 2.3, 4.3, 7.3).

Thin, thread-safe façade around :class:`paxini_usb_core.UsbSensorWorker`.  One
:class:`SensorLink` per physical finger (Left / Right).  The GUI never touches
the worker's internals directly - it polls :meth:`SensorLink.reading`, which
returns a plain snapshot of the latest frame.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional

import numpy as np

from . import _paths  # noqa: F401
import paxini_usb_core as core

# Line rate for the sensor boards. This is deliberately the same value
# ``paxini_usb_core`` opens the port with (its own ``BAUDRATE`` constant), so
# the GUI can never disagree with the reference reader - a mismatch here means
# the board never answers and the link reports "no sensor answered".
# It stays editable per role in the Connection panel.
SENSOR_BAUDRATE = core.BAUDRATE

# Device address: 0 asks the core to auto-detect it on connect.
ADDR_AUTO = 0

STATE_DISCONNECTED = "disconnected"
STATE_CONNECTING = "connecting"
STATE_STREAMING = "streaming"
STATE_ERROR = "error"


@dataclass
class SensorReading:
    """Snapshot of one sensor frame (values are NaN where undefined)."""

    t: float
    epoch: float
    fx: float
    fy: float
    fz: float
    f_mag: float
    f_shear: float
    mu: float
    cop_x: float
    cop_y: float
    shear_dir: float
    tilt: float
    peak_taxel: float
    peak_fz: float
    contact_taxels: float
    taxels: Optional[np.ndarray]  # (52, 3) float32 [Fx, Fy, Fz] or None


class SensorLink:
    """Lifecycle + snapshot access for one tactile sensor."""

    def __init__(
        self,
        tag: str,
        port: str,
        log: Callable[[str], None],
        baudrate: int = SENSOR_BAUDRATE,
        device_addr: int = ADDR_AUTO,
        model: str = "S2015_elite",
        worker_factory: Optional[Callable[..., core.UsbSensorWorker]] = None,
    ) -> None:
        self.tag = tag                      # "L" or "R"
        self.port = port
        self.baudrate = int(baudrate)
        # 0 = auto-detect; 0x01..0x08 pins the address (still tried first).
        self.device_addr = int(device_addr)
        self.model = model
        self._log = log
        self._factory = worker_factory or core.UsbSensorWorker
        self.worker: Optional[core.UsbSensorWorker] = None
        self._last_error = ""

    # -- geometry -----------------------------------------------------------
    @property
    def xy(self) -> np.ndarray:
        return core.S2015_ELITE_XYZ[:, :2]

    @property
    def xyz(self) -> np.ndarray:
        return core.S2015_ELITE_XYZ

    @property
    def n_points(self) -> int:
        return 52

    @property
    def label(self) -> str:
        return "Left" if self.tag == "L" else "Right"

    # -- lifecycle ----------------------------------------------------------
    def connect(self) -> None:
        if self.is_active():
            return
        self.worker = self._factory(
            self.port,
            self._log,
            device_addr=self.device_addr,
            model=self.model,
            baudrate=self.baudrate,
        )
        self._last_error = ""
        self.worker.start()
        self._log(f"[{self.tag}] {self.port} @ {self.baudrate} baud, "
                  + (f"address 0x{self.device_addr:02X}"
                     if self.device_addr else "address auto-detect"))

    def disconnect(self, wait: bool = False, timeout: float = 1.5) -> int:
        """Stop the worker, flush any recording, and return the row count."""
        rows = 0
        w = self.worker
        if w is not None:
            rec = getattr(w, "recorder", None)
            if rec is not None:
                try:
                    rows = rec.stop()
                except Exception:
                    rows = 0
                w.recorder = None
            w.stop()
            if wait:
                w.join(timeout=timeout)
            if w.is_alive():
                return rows
        self.worker = None
        return rows

    def is_active(self) -> bool:
        return self.worker is not None and self.worker.is_alive()

    # -- status -------------------------------------------------------------
    @property
    def detected_addr(self) -> int:
        """Bus address the board actually answered on (0 until detected)."""
        w = self.worker
        return int(getattr(w, "device_addr", 0)) if w is not None else 0

    @property
    def addr_text(self) -> str:
        addr = self.detected_addr
        return f"0x{addr:02X}" if addr else "auto"

    @property
    def state(self) -> str:
        w = self.worker
        if w is None:
            return STATE_DISCONNECTED
        if w.state == "error":
            return STATE_ERROR
        if w.state == "streaming":
            return STATE_STREAMING
        if w.state == "connecting" and w.is_alive():
            return STATE_CONNECTING
        return STATE_DISCONNECTED

    @property
    def error(self) -> str:
        return self.worker.error if self.worker is not None else self._last_error

    @property
    def rate_hz(self) -> float:
        return self.worker.rate_hz if self.worker is not None else 0.0

    @property
    def frames(self) -> int:
        return self.worker.frames if self.worker is not None else 0

    @property
    def timeouts(self) -> int:
        """Frame-drop counter (missing resultant/distributed responses)."""
        return getattr(self.worker, "timeouts", 0)

    def health(self) -> Dict[str, float]:
        total = self.frames + self.timeouts
        return {
            "rate_hz": self.rate_hz,
            "frames": float(self.frames),
            "drops": float(self.timeouts),
            "drop_pct": (100.0 * self.timeouts / total) if total else 0.0,
        }

    # -- tare (Phase 4.3) ---------------------------------------------------
    def request_tare(self, n_frames: int = 50) -> None:
        if self.worker is not None:
            self.worker.request_tare(n_frames)
            self._log(f"[{self.tag}] tare requested ({n_frames} frames)")

    def reset_tare(self) -> None:
        if self.worker is not None:
            self.worker.reset_tare()
            self._log(f"[{self.tag}] tare reset")

    # -- data ---------------------------------------------------------------
    def reading(self) -> Optional[SensorReading]:
        w = self.worker
        if w is None or w.latest_row is None:
            return None
        r = w.latest_row
        C = core.COL
        tax = w.latest_taxels
        return SensorReading(
            t=float(r[0]),
            epoch=0.0,
            fx=float(r[C["Fx"]]),
            fy=float(r[C["Fy"]]),
            fz=float(r[C["Fz"]]),
            f_mag=float(r[C["F_mag"]]),
            f_shear=float(r[C["F_shear"]]),
            mu=float(r[C["mu"]]),
            cop_x=float(r[C["CoP_x"]]),
            cop_y=float(r[C["CoP_y"]]),
            shear_dir=float(r[C["shear_dir"]]),
            tilt=float(r[C["tilt"]]),
            peak_taxel=float(r[C["peak_idx"]]),
            peak_fz=float(r[C["peak_Fz"]]),
            contact_taxels=float(r[C["contact_n"]]),
            taxels=None if tax is None else np.array(tax, dtype=np.float32, copy=True),
        )

    def raw_row(self):
        """Latest 15-column metric row (or None) - used by the recorder."""
        w = self.worker
        return None if w is None else w.latest_row

    def raw_taxels(self) -> Optional[np.ndarray]:
        w = self.worker
        return None if w is None else w.latest_taxels

    def ring_window(self, t_from: float, cols) -> np.ndarray:
        w = self.worker
        if w is None:
            return np.empty((0, len(cols)))
        return w.ring.window(t_from, list(cols))

    # -- recording hook -----------------------------------------------------
    def attach_recorder(self, recorder) -> None:
        if self.worker is not None:
            self.worker.recorder = recorder

    def detach_recorder(self):
        w = self.worker
        if w is None:
            return None
        rec = getattr(w, "recorder", None)
        w.recorder = None
        return rec
