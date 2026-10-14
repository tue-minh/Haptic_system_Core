"""Synchronized data recording & CSV logging (Phase 5).

Two artefacts are produced per session, both in the chosen output directory:

1. ``data_record_<stamp>.csv`` - the *master* file.  One row per sampling tick
   containing the stepper coordinates, the gripper state and the full metric +
   52-taxel blocks for **both** sensors in one synchronized schema.
2. ``paxini_<stamp>_<L|R>_<port>.csv`` - one file per sensor, byte-compatible
   with the offline analyser :mod:`plot_paxini_data` (which auto-discovers files
   named ``paxini_YYYYMMDD_HHMMSS_[LR]_<port>.csv``).  These are written by the
   existing :class:`paxini_usb_core.CsvRecorder` directly from the sensor
   threads, so no frames are lost.

Throughput: the master recorder samples on its own thread at ``rate_hz`` (100 Hz
by default), pushes rows into a non-blocking ``queue.SimpleQueue`` and flushes
them to disk in batches from a second thread.  File I/O never blocks the serial
readers or the GUI event loop.
"""
from __future__ import annotations

import datetime
import os
import queue
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from . import _paths  # noqa: F401
import paxini_usb_core as core
from .sensors import SensorLink

DEFAULT_RECORD_RATE_HZ = 100.0

# Master-file leading columns (before the per-sensor blocks).
LEAD_COLUMNS = [
    "t_rel_s",
    "epoch_s",
    "Stepper_X_mm",
    "Stepper_Y_mm",
    "Stepper_Z_mm",
    "Gripper_Pos_raw",
    "Gripper_Volt_V",
]
N_LEAD = len(LEAD_COLUMNS) - 2  # values supplied by sample_fn (everything but t/epoch)

# The 13 per-sensor metrics, in ring-buffer order (t excluded).
SENSOR_METRICS = list(core.METRIC_COLUMNS[1:])


def sensor_block_columns(tag: str) -> List[str]:
    cols = [f"{tag}_{name}" for name in SENSOR_METRICS]
    for k in range(1, 53):
        cols += [f"{tag}_t{k}_Fx", f"{tag}_t{k}_Fy", f"{tag}_t{k}_Fz"]
    return cols


def master_columns() -> List[str]:
    return LEAD_COLUMNS + sensor_block_columns("L") + sensor_block_columns("R")


def _fmt_list() -> List[str]:
    fmt = ["%.6f", "%.6f", "%.4f", "%.4f", "%.4f", "%.0f", "%.2f"]
    for _ in range(2):
        fmt += ["%.3f"] * len(SENSOR_METRICS)
        fmt += ["%.1f"] * (3 * 52)
    return fmt


# Sensor block value count: 13 metrics + 156 taxel values.
BLOCK_VALUES = len(SENSOR_METRICS) + 3 * 52


def nan_block() -> List[float]:
    return [float("nan")] * BLOCK_VALUES


def sensor_block(link: Optional[SensorLink]) -> List[float]:
    """Flatten one sensor's latest metrics + taxels into a single value list."""
    if link is None:
        return nan_block()
    row = link.raw_row()
    tax = link.raw_taxels()
    if row is None:
        return nan_block()
    values = [float(v) for v in row[1:]]  # drop the host timestamp
    if len(values) < len(SENSOR_METRICS):
        values += [float("nan")] * (len(SENSOR_METRICS) - len(values))
    if tax is not None and tax.size == 3 * 52:
        values += [float(v) for v in np.asarray(tax, dtype=np.float64).ravel()]
    else:
        values += [float("nan")] * (3 * 52)
    return values


class UnifiedRecorder:
    """Master CSV writer: sampler thread -> queue -> batched writer thread."""

    def __init__(
        self,
        path: str,
        t0: float,
        sample_fn: Callable[[], List[float]],
        rate_hz: float = DEFAULT_RECORD_RATE_HZ,
        meta: str = "",
    ) -> None:
        self.path = path
        self.t0 = t0
        self.rate_hz = max(1.0, float(rate_hz))
        self._sample_fn = sample_fn
        self.columns = master_columns()
        self.ncols = len(self.columns)
        self.rows = 0
        self.locked = False

        self._q: "queue.SimpleQueue" = queue.SimpleQueue()
        self._stop_evt = threading.Event()
        self._f = open(path, "w", newline="", encoding="utf-8")
        if meta:
            self._f.write(f"# {meta}\n")
        self._f.write(",".join(self.columns) + "\n")
        self._fmt = _fmt_list()

        self._sampler = threading.Thread(target=self._sample_loop, daemon=True,
                                         name="UnifiedRecorder-sampler")
        self._writer = threading.Thread(target=self._write_loop, daemon=True,
                                        name="UnifiedRecorder-writer")
        self._sampler.start()
        self._writer.start()

    # -- sampler ------------------------------------------------------------
    def _sample_loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_t = time.perf_counter()
        while not self._stop_evt.is_set():
            now = time.perf_counter()
            if now < next_t:
                self._stop_evt.wait(next_t - now)
                if self._stop_evt.is_set():
                    break
            next_t += period
            if next_t < now:  # fell behind: resynchronise instead of flooding
                next_t = now + period
            try:
                values = list(self._sample_fn())
            except Exception:
                values = [float("nan")] * N_LEAD + nan_block() + nan_block()
            if len(values) < N_LEAD + 2 * BLOCK_VALUES:
                values += [float("nan")] * (N_LEAD + 2 * BLOCK_VALUES - len(values))
            row = [core.now() - self.t0, time.time()] + values
            self._q.put(row)
        self._q.put(None)

    # -- writer -------------------------------------------------------------
    def _write_loop(self) -> None:
        done = False
        while not done:
            try:
                item = self._q.get(timeout=0.25)
            except queue.Empty:
                continue
            batch = []
            while True:
                if item is None:
                    done = True
                    break
                batch.append(item)
                if len(batch) >= 5000:
                    break
                try:
                    item = self._q.get_nowait()
                except queue.Empty:
                    break
            if not batch:
                continue
            arr = np.asarray(batch, dtype=np.float64)
            np.savetxt(self._f, arr, fmt=self._fmt, delimiter=",")
            self.rows += len(batch)
        self._f.flush()

    def stop(self) -> int:
        self._stop_evt.set()
        self._sampler.join(timeout=5)
        self._writer.join(timeout=10)
        try:
            self._f.close()
        except Exception:
            pass
        return self.rows


class RecordingManager:
    """Owns the master recorder plus the per-sensor offline-format recorders."""

    def __init__(self, log: Callable[[str], None], output_dir: str) -> None:
        self._log = log
        self.output_dir = output_dir
        self.active = False
        self.stamp = ""
        self.t0 = 0.0
        self.rate_hz = DEFAULT_RECORD_RATE_HZ
        self.master: Optional[UnifiedRecorder] = None
        self.files: List[str] = []
        self._sensor_recorders: List[Tuple[SensorLink, core.CsvRecorder]] = []
        self.master_rows = 0
        self.total_rows = 0

    # -- configuration ------------------------------------------------------
    def set_output_dir(self, path: str) -> None:
        self.output_dir = path

    def ensure_output_dir(self) -> bool:
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            return True
        except OSError as exc:
            self._log(f"[rec] cannot create {self.output_dir}: {exc}")
            return False

    @property
    def elapsed(self) -> float:
        return (core.now() - self.t0) if self.active else 0.0

    # -- lifecycle ----------------------------------------------------------
    def start(
        self,
        links: Dict[str, SensorLink],
        motor_snapshot: Callable[[], object],
        rate_hz: float = DEFAULT_RECORD_RATE_HZ,
        motor_port: str = "",
    ) -> bool:
        if self.active:
            return True
        if not self.ensure_output_dir():
            return False

        streaming = {tag: link for tag, link in links.items() if link.state == "streaming"}
        if not streaming:
            self._log("[rec] no streaming sensor to record")
            return False

        self.stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.t0 = core.now()
        self.rate_hz = float(rate_hz)
        self.files = []
        self._sensor_recorders = []

        def sample_fn() -> List[float]:
            snap = motor_snapshot()
            if snap is not None and getattr(snap, "connected", False):
                lead = [snap.x, snap.y, snap.z, float(snap.servo_pos), snap.servo_volt_v]
            else:
                lead = [float("nan")] * N_LEAD
            return lead + sensor_block(links.get("L")) + sensor_block(links.get("R"))

        master_path = os.path.join(self.output_dir, f"data_record_{self.stamp}.csv")
        meta = (
            f"unified recording stamp={self.stamp} rate={self.rate_hz:g}Hz "
            f"motor_port={motor_port}"
        )
        try:
            self.master = UnifiedRecorder(master_path, self.t0, sample_fn,
                                          rate_hz=self.rate_hz, meta=meta)
        except OSError as exc:
            self._log(f"[rec] cannot create {master_path}: {exc}")
            return False
        self.files.append(master_path)

        for tag, link in streaming.items():
            fname = f"paxini_{self.stamp}_{tag}_{link.port}.csv"
            path = os.path.join(self.output_dir, fname)
            try:
                rec = core.CsvRecorder(
                    path, link.n_points, self.t0,
                    meta=f"sensor={tag} port={link.port} model={link.model}",
                )
            except OSError as exc:
                self._log(f"[rec] cannot create {path}: {exc}")
                continue
            link.attach_recorder(rec)
            self._sensor_recorders.append((link, rec))
            self.files.append(path)

        self.active = True
        self._log(
            f"[rec] started -> {os.path.basename(master_path)} "
            f"({len(self._sensor_recorders)} sensor file(s) @ {self.rate_hz:g} Hz)"
        )
        return True

    def stop(self) -> Dict[str, object]:
        if not self.active:
            return {"rows": 0, "files": [], "elapsed": 0.0}

        for link, _rec in self._sensor_recorders:
            link.detach_recorder()
        sensor_rows = 0
        for _link, rec in self._sensor_recorders:
            sensor_rows += rec.stop()
        self._sensor_recorders = []

        master_rows = 0
        if self.master is not None:
            master_rows = self.master.stop()
        self.master = None

        elapsed = core.now() - self.t0
        self.active = False
        self.master_rows = master_rows
        self.total_rows = master_rows
        self._log(
            f"[rec] stopped after {elapsed:6.1f} s - master {master_rows} rows, "
            f"{sensor_rows} sensor rows, {len(self.files)} file(s)"
        )
        return {"rows": master_rows, "sensor_rows": sensor_rows,
                "files": list(self.files), "elapsed": elapsed}
