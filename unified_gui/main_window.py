"""Main application window: layout, refresh loop and lifecycle wiring (Phase 6.1).

Layout
------
Left column  : Connection, Stepper Control, Gripper, Data Recording.
Right column : force-history plot + live metrics grid, above the 3D tactile view.
Bottom       : rolling log.

All hardware access happens on worker threads; this window only submits commands
and polls telemetry snapshots from a single Qt timer.
"""
from __future__ import annotations

import datetime
import os
import queue
from typing import Dict, Optional

import numpy as np

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt

from . import _paths  # noqa: F401
from ._paths import REPO_ROOT
import paxini_usb_core as core
from .deps import verify
from .haptic import HapticPayloadSynthesizer, HapticSnapshot, Stm32HapticStreamer, TueBinaryEncoder
from .motor import MotorWorker
from .recording import RecordingManager
from .sensors import SensorLink, STATE_CONNECTING, STATE_ERROR, STATE_STREAMING
from .plot3d import Tactile3DView
from .taxelmap import TaxelMapView
from .valvemap import ValveMapView
from .panels.connection import ConnectionPanel, ROLE_LABELS
from .panels.motor import GripperPanel, StepperPanel
from .panels.recording import RecordingPanel
from .panels.sensor import SensorMonitorPanel

REFRESH_FPS = 30
RECORD_RATE_HZ = 100.0
DEFAULT_OUTPUT_DIR = os.path.join(REPO_ROOT, "Unified_Control")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Unified Motor Control & Dual PaXini Tactile Acquisition")
        self.resize(1680, 980)

        self._log_q: "queue.SimpleQueue[str]" = queue.SimpleQueue()
        self.motor: Optional[MotorWorker] = None
        self.links: Dict[str, SensorLink] = {}
        self.haptic: Optional[Stm32HapticStreamer] = None
        self._haptic_preview = HapticPayloadSynthesizer()
        self._haptic_preview_enc = TueBinaryEncoder()
        self.recorder = RecordingManager(self.post_log, DEFAULT_OUTPUT_DIR)
        self._tick = 0

        self._build_ui()
        self._refresh_ports(initial=True)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(1000 / REFRESH_FPS))
        self.timer.timeout.connect(self._on_tick)
        self.timer.start()

        report = verify()
        self.post_log(f"environment: {'READY' if report.ok else 'MISSING PACKAGES'}"
                      + ("" if report.opengl_ok else " (OpenGL unavailable - Matplotlib 3D fallback)"))
        if not report.ok:
            for chk in report.missing:
                self.post_log(f"  missing: {chk.package} ({chk.purpose})")
        if not report.opengl_ok:
            self.post_log("3D view: Matplotlib 3D is CPU-rendered; lower its 'Update' "
                          "rate or untick 'Live' if the GUI feels sluggish.")
        self.post_log(f"recordings will be written to: {DEFAULT_OUTPUT_DIR}")

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(8, 8, 8, 8)

        main_split = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        top_split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)

        # ---- left column -------------------------------------------------
        left_host = QtWidgets.QWidget()
        left = QtWidgets.QVBoxLayout(left_host)
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(10)

        self.connection = ConnectionPanel()
        self.connection.connect_requested.connect(self._connect_role)
        self.connection.disconnect_requested.connect(self._disconnect_role)
        self.connection.refresh_requested.connect(self._refresh_ports)
        self.connection.connect_all_requested.connect(self._connect_all)
        self.connection.disconnect_all_requested.connect(self._disconnect_all)

        self.stepper = StepperPanel(self.post_log)
        self.gripper = GripperPanel(self.post_log)
        self.record_panel = RecordingPanel(DEFAULT_OUTPUT_DIR)
        self.record_panel.start_requested.connect(self._start_recording)
        self.record_panel.stop_requested.connect(self._stop_recording)
        self.record_panel.output_dir_changed.connect(self.recorder.set_output_dir)

        for widget in (self.connection, self.stepper, self.gripper, self.record_panel):
            left.addWidget(widget)
        left.addStretch(1)

        left_scroll = QtWidgets.QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left_host)
        left_scroll.setMinimumWidth(470)
        top_split.addWidget(left_scroll)

        # ---- right column ------------------------------------------------
        right_split = QtWidgets.QSplitter(Qt.Orientation.Vertical)

        self.sensor_monitor = SensorMonitorPanel(self.post_log)
        right_split.addWidget(self.sensor_monitor)

        surface_box = QtWidgets.QGroupBox("Tactile surface & haptic output")
        sv = QtWidgets.QVBoxLayout(surface_box)
        self.surface_tabs = QtWidgets.QTabWidget()
        self.taxel_map = TaxelMapView({})
        self.view3d = Tactile3DView({})
        self.valve_map = ValveMapView()
        self.surface_tabs.addTab(self.taxel_map, "Taxel map (live)")
        self.surface_tabs.addTab(self.view3d, "3D cloud")
        self.surface_tabs.addTab(self.valve_map, "Haptic valves (5)")
        self.surface_tabs.setCurrentIndex(0)
        sv.addWidget(self.surface_tabs)
        right_split.addWidget(surface_box)
        right_split.setStretchFactor(0, 3)
        right_split.setStretchFactor(1, 4)

        top_split.addWidget(right_split)
        top_split.setStretchFactor(0, 0)
        top_split.setStretchFactor(1, 1)
        main_split.addWidget(top_split)

        # ---- log ---------------------------------------------------------
        log_box = QtWidgets.QGroupBox("Log")
        lb = QtWidgets.QVBoxLayout(log_box)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1000)
        self.log_view.setFont(QtGui.QFont("Consolas", 9))
        lb.addWidget(self.log_view)
        main_split.addWidget(log_box)
        main_split.setStretchFactor(0, 1)
        main_split.setSizes([780, 170])

        outer.addWidget(main_split, 1)

        self.status = self.statusBar()
        self.lbl_status = QtWidgets.QLabel("Not connected")
        self.lbl_status.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                                      QtWidgets.QSizePolicy.Policy.Preferred)
        self.status.addPermanentWidget(self.lbl_status)

        QtGui.QShortcut(QtGui.QKeySequence("Escape"), self, activated=self._emergency_stop)
        QtGui.QShortcut(QtGui.QKeySequence("Ctrl+R"), self, activated=self.record_panel._on_toggle)

    # ------------------------------------------------------------- logging
    def post_log(self, msg: str) -> None:
        self._log_q.put(f"{datetime.datetime.now():%H:%M:%S}  {msg}")

    def _drain_log(self) -> None:
        lines = []
        try:
            while True:
                lines.append(self._log_q.get_nowait())
        except queue.Empty:
            pass
        if lines:
            self.log_view.appendPlainText("\n".join(lines))

    # --------------------------------------------------------------- ports
    def _refresh_ports(self, initial: bool = False) -> None:
        ports = self.connection.scan_ports(auto_assign=initial)
        devices = ", ".join(self.connection.port_devices()) or "none"
        self.post_log(f"ports: {devices}")
        if initial:
            self.connection.set_state("motor", "off", "-")
            self.connection.set_state("sensor_l", "off", "-")
            self.connection.set_state("sensor_r", "off", "-")
            assigned = [self.connection.selected_port(r) for r in ("motor", "sensor_l", "sensor_r")]
            if any(assigned):
                self.post_log("auto-assigned ports: "
                              + ", ".join(f"{r}={p or '-'}"
                                          for r, p in zip(("motor", "sensor_l", "sensor_r"), assigned)))

    # ------------------------------------------------------------ connect
    def _connect_role(self, role: str) -> None:
        port = self.connection.selected_port(role)
        if not port:
            QtWidgets.QMessageBox.warning(self, "No port", f"No COM port selected for {ROLE_LABELS[role]}.")
            return
        # Port collision guard (also covers the Connect-All path, which does
        # not go through the panel button handler).
        conflict = self.connection.collision(role)
        if conflict:
            self.post_log(
                f"port {port} is already used by {ROLE_LABELS[conflict]} - "
                f"{ROLE_LABELS[role]} not connected")
            QtWidgets.QMessageBox.warning(
                self, "Port collision",
                f"{port} is already used by {ROLE_LABELS[conflict]}.\nPlease pick a "
                "different port for " + ROLE_LABELS[role] + ".")
            return
        baud = self.connection.selected_baud(role)
        if role == "motor":
            self._connect_motor(port, baud)
        elif role == "haptic":
            self._connect_haptic(port, baud)
        else:
            self._connect_sensor("L" if role == "sensor_l" else "R", port, baud)

    def _connect_motor(self, port: str, baud: int) -> None:
        if self.motor is not None:
            return
        self.motor = MotorWorker(port, baudrate=baud, log=self.post_log)
        self.motor.start()
        self.stepper.bind(self.motor)
        self.gripper.bind(self.motor)
        self.connection.set_state("motor", "connecting", f"{port} @ {baud}")
        self.post_log(f"connecting motor board on {port} @ {baud} baud")

    def _connect_sensor(self, tag: str, port: str, baud: int) -> None:
        if tag in self.links:
            return
        link = SensorLink(tag, port, self.post_log, baudrate=baud)
        self.links[tag] = link
        link.connect()
        role = "sensor_l" if tag == "L" else "sensor_r"
        self.connection.set_state(role, "connecting", f"{port} @ {baud}")
        self._rebind_sensor_views()
        self.post_log(f"connecting {link.label} sensor on {port} @ {baud} baud")

    def _connect_haptic(self, port: str, baud: int) -> None:
        if self.haptic is not None:
            return
        streamer = Stm32HapticStreamer(
            port, baudrate=baud, log=self.post_log, clock=core.now)
        self.haptic = streamer
        streamer.start()
        self.connection.set_state("haptic", "connecting", f"{port} @ {baud}")
        self.post_log(f"connecting haptic stream on {port} @ {baud} baud (CAN 0x300, 100 Hz)")

    def _connect_all(self) -> None:
        for role in ("motor", "sensor_l", "sensor_r", "haptic"):
            if not self.connection.is_connected(role):
                self._connect_role(role)

    def _disconnect_role(self, role: str) -> None:
        if role == "motor":
            self._disconnect_motor()
        elif role == "haptic":
            self._disconnect_haptic()
        else:
            tag = "L" if role == "sensor_l" else "R"
            self._disconnect_sensor(tag)

    def _disconnect_motor(self) -> None:
        worker, self.motor = self.motor, None
        if worker is not None:
            worker.shutdown()
            worker.join(timeout=1.5)
            worker.close_client()
        self.stepper.bind(None)
        self.gripper.bind(None)
        self.connection.set_state("motor", "off", "-")
        self.post_log("motor board disconnected")

    def _disconnect_haptic(self) -> None:
        streamer, self.haptic = self.haptic, None
        if streamer is not None:
            streamer.shutdown()
            streamer.join(timeout=1.5)
        self.connection.set_state("haptic", "off", "-")
        self.post_log("haptic stream disconnected")

    def _disconnect_sensor(self, tag: str) -> None:
        if self.recorder.active:
            QtWidgets.QMessageBox.information(
                self, "Recording", "Stop recording before disconnecting a sensor.")
            return
        link = self.links.pop(tag, None)
        role = "sensor_l" if tag == "L" else "sensor_r"
        if link is not None:
            link.disconnect(wait=True)
        self.connection.set_state(role, "off", "-")
        self._rebind_sensor_views()
        self.post_log(f"{'Left' if tag == 'L' else 'Right'} sensor disconnected")

    def _disconnect_all(self) -> None:
        if self.recorder.active:
            self._stop_recording()
        for tag in list(self.links):
            link = self.links.pop(tag, None)
            if link is not None:
                link.disconnect(wait=True)
            role = "sensor_l" if tag == "L" else "sensor_r"
            self.connection.set_state(role, "off", "-")
        self._disconnect_motor()
        self._disconnect_haptic()
        self._rebind_sensor_views()

    def _rebind_sensor_views(self) -> None:
        self.sensor_monitor.bind(self.links)
        self.taxel_map.set_links(self.links)
        self.view3d.set_links(self.links)

    def _emergency_stop(self) -> None:
        if self.motor is not None:
            self.motor.abort()
            self.post_log("EMERGENCY STOP (Escape)")

    # ----------------------------------------------------------- recording
    def _start_recording(self) -> None:
        self.recorder.set_output_dir(self.record_panel.output_dir)
        ok = self.recorder.start(
            self.links,
            motor_snapshot=(self.motor.snapshot if self.motor is not None else (lambda: None)),
            rate_hz=self.record_panel.rate_hz,
            motor_port=(self.motor.port if self.motor is not None else ""),
        )
        if not ok:
            QtWidgets.QMessageBox.warning(
                self, "Recording",
                "Cannot start recording: no streaming sensor (or the output "
                "directory is not writable).")
            self.record_panel.set_recording(False)
            return
        self.record_panel.set_recording(True)
        self.record_panel.update_status(0.0, 0, self.recorder.files)

    def _stop_recording(self) -> None:
        summary = self.recorder.stop()
        self.record_panel.set_recording(False)
        files = summary.get("files", [])
        self.record_panel.update_status(
            elapsed=float(summary.get("elapsed", 0.0)),
            rows=int(summary.get("rows", 0)),
            files=files,
            note="No file yet",
        )

    # ---------------------------------------------------------------- tick
    def _on_tick(self) -> None:
        self._tick += 1
        self._drain_log()

        snap = self.motor.snapshot() if self.motor is not None else None

        if self._tick % 3 == 0:
            self._update_connection_states(snap)
            if snap is not None:
                self.stepper.update_telemetry(snap)
                self.gripper.update_telemetry(snap)

        now = self._now()
        self.sensor_monitor.refresh(now)

        readings = {tag: link.reading() for tag, link in self.links.items()}
        self.taxel_map.update_readings(readings)
        # the Matplotlib 3D canvas is CPU-rendered; only pay for it when shown
        if self.surface_tabs.currentWidget() is self.view3d:
            self.view3d.update_readings(readings)

        self._update_haptic(readings)

        if self._tick % 15 == 0:
            self._update_recording_status()
            self._maybe_autostop_recording()
            self._update_status_bar(snap)

    # --------------------------------------------------------------- haptic
    @staticmethod
    def _haptic_side(reading):
        """Flatten one sensor snapshot into the synthesizer's input contract."""
        if reading is None:
            return None
        tax = reading.taxels
        fz = None if tax is None else np.asarray(tax, dtype=float)[:, 2]
        return {
            "fz": reading.fz,
            "fy": reading.fy,
            "fx": reading.fx,
            "cop_x": reading.cop_x,
            "cop_y": reading.cop_y,
            "taxels_fz": fz,
            "t": reading.t,
        }

    def _update_haptic(self, readings) -> None:
        left = self._haptic_side(readings.get("L"))
        right = self._haptic_side(readings.get("R"))
        tare = any(link.tare_active for link in self.links.values())

        if self.haptic is not None:
            self.haptic.update_sensors(left, right, tare_active=tare)
            self.valve_map.update_snapshot(self.haptic.snapshot())
            return

        # No output link: still show what would be sent, so the tactile ->
        # valve mapping stays visible while wiring the STM32 output.
        now = core.now()
        payload = self._haptic_preview.synthesize(left, right, tare_active=tare, now=now)
        frame = self._haptic_preview_enc.encode(payload)
        self.valve_map.update_snapshot(HapticSnapshot.from_payload(
            payload, now, state="preview (no output link)", connected=False,
            seq=self._haptic_preview_enc.seq - 1, frame=frame))

    def _update_recording_status(self) -> None:
        if self.recorder.active:
            rows = self.recorder.master.rows if self.recorder.master is not None else 0
            self.record_panel.update_status(
                elapsed=self.recorder.elapsed, rows=rows, files=self.recorder.files,
            )

    def _maybe_autostop_recording(self) -> None:
        if self.recorder.active and not any(
            link.state == STATE_STREAMING for link in self.links.values()
        ):
            self.post_log("recording auto-stopped: no streaming sensor left")
            self._stop_recording()

    def _update_connection_states(self, snap) -> None:
        # motor
        if self.motor is None:
            pass
        elif snap is not None:
            if snap.error:
                self.connection.set_state("motor", "error", snap.error[:40])
            elif snap.connected:
                self.connection.set_state(
                    "motor", "ready",
                    f"{snap.board} fw{snap.firmware} | {snap.poll_hz:4.1f} Hz")
            elif snap.connecting:
                self.connection.set_state("motor", "connecting", self.motor.port)
            else:
                self.connection.set_state("motor", "off", "-")

        # sensors
        for tag, link in self.links.items():
            role = "sensor_l" if tag == "L" else "sensor_r"
            state = link.state
            if state == STATE_STREAMING:
                health = link.health()
                # the detected bus address leads, so it survives clipping
                text = (f"ID {link.addr_text} | {health['rate_hz']:4.1f} Hz | drops "
                        f"{int(health['drops'])} ({health['drop_pct']:.1f}%)")
                self.connection.set_state(role, "ready", text)
            elif state == STATE_CONNECTING:
                self.connection.set_state(role, "connecting",
                                          f"{link.port} | detecting ID…")
            elif state == STATE_ERROR:
                self.connection.set_state(role, "error", link.error[:40])
            else:
                self.connection.set_state(role, "off", "-")

        # haptic output link
        if self.haptic is not None:
            hs = self.haptic.snapshot()
            if hs.error:
                self.connection.set_state("haptic", "error", hs.error[:40])
            elif hs.state == "streaming" and hs.sent:
                self.connection.set_state(
                    "haptic", "ready",
                    f"CAN 0x300 | {hs.rate_hz:4.1f} Hz | {hs.sent} frames")
            elif hs.state == "streaming":
                self.connection.set_state("haptic", "connecting", self.haptic.port)
            else:
                self.connection.set_state("haptic", "off", "-")

    def _update_status_bar(self, snap) -> None:
        parts = []
        if snap is not None and snap.connected:
            parts.append(f"motor {'BUSY' if snap.busy else 'idle'}"
                         f" ({snap.x:6.1f},{snap.y:6.1f},{snap.z:6.1f}) mm")
        else:
            parts.append("motor offline")
        for tag, link in self.links.items():
            label = "L" if tag == "L" else "R"
            parts.append(f"{label}:{link.rate_hz:4.1f}Hz" if link.state == STATE_STREAMING
                         else f"{label}:off")
        if self.haptic is not None:
            hs = self.haptic.snapshot()
            parts.append(f"haptic:{hs.rate_hz:4.1f}Hz" if hs.state == "streaming"
                         else "haptic:off")
        if self.recorder.active:
            parts.append(f"REC {self.recorder.elapsed:6.1f}s")
        self.lbl_status.setText("   |   ".join(parts))

    @staticmethod
    def _now() -> float:
        return core.now()

    # ------------------------------------------------------------ watchdog
    def changeEvent(self, ev) -> None:  # noqa: N802
        if ev.type() == QtCore.QEvent.Type.WindowStateChange and self.isMinimized():
            self.stepper.release_jogs()
            self.post_log("window minimised - jog released (safety)")
        elif ev.type() == QtCore.QEvent.Type.ActivationChange and not self.isActiveWindow():
            self.stepper.release_jogs()
        super().changeEvent(ev)

    def closeEvent(self, ev) -> None:  # noqa: N802
        self.timer.stop()
        self.connection.lock_all(True)
        if self.recorder.active:
            self._stop_recording()
        for tag in list(self.links):
            link = self.links.pop(tag, None)
            if link is not None:
                link.disconnect(wait=True)
        self.stepper.bind(None)
        if self.haptic is not None:
            self.haptic.shutdown()
            self.haptic.join(timeout=1.5)
            self.haptic = None
        if self.motor is not None:
            self.motor.shutdown()
            self.motor.join(timeout=1.5)
            self.motor.close_client()
            self.motor = None
        super().closeEvent(ev)
