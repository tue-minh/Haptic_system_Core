"""Stepper (CoreXY + Z) and gripper (STS3215) control panels (Phase 3).

Both panels are pure views over a shared :class:`MotorWorker`: they read the
worker's telemetry snapshot on the GUI refresh tick and submit commands back.
All motion controls are hard-disabled unless the motor link is connected *and*
the drivers are energised (Phase 7.2 watchdog).
"""
from __future__ import annotations

from typing import Callable, Optional

from PyQt6 import QtCore, QtWidgets
from PyQt6.QtCore import Qt, pyqtSignal

from .. import theme
from ..motor import (
    ACCEL_MAX,
    ACCEL_MIN,
    GRIPPER_ACCEL_MAX,
    GRIPPER_ACCEL_MIN,
    GRIPPER_CLOSE,
    GRIPPER_OPEN,
    GRIPPER_POS_MAX,
    GRIPPER_POS_MIN,
    VEL_MAX,
    VEL_MIN,
    MotorSnapshot,
    MotorWorker,
    clamp,
)
from ..widgets import fmt, value_label


class JogButton(QtWidgets.QPushButton):
    """Press-and-hold jog button with a leave/deactivate safety release."""

    jog_pressed = pyqtSignal(str, float)
    jog_released = pyqtSignal()

    def __init__(self, text: str, axis: str, sign: float, parent=None) -> None:
        super().__init__(text, parent)
        self.setObjectName("jog")
        self.axis = axis
        self.sign = sign
        self._down = False

    def mousePressEvent(self, ev) -> None:  # noqa: N802
        if ev.button() == Qt.MouseButton.LeftButton:
            self._down = True
            self.jog_pressed.emit(self.axis, self.sign)
        super().mousePressEvent(ev)

    def mouseReleaseEvent(self, ev) -> None:  # noqa: N802
        self._release()
        super().mouseReleaseEvent(ev)

    def leaveEvent(self, ev) -> None:  # noqa: N802
        # watchdog: pointer left the button while held -> stop immediately
        self._release()
        super().leaveEvent(ev)

    def is_down(self) -> bool:
        return self._down

    def force_release(self) -> None:
        self._release()

    def _release(self) -> None:
        if self._down:
            self._down = False
            self.jog_released.emit()


class StepperPanel(QtWidgets.QGroupBox):
    """Realtime telemetry, enable toggle, feed tuning and X/Y/Z jogging."""

    def __init__(self, log: Callable[[str], None], parent=None) -> None:
        super().__init__("Stepper Control  (CoreXY + Z)", parent)
        self._log = log
        self.worker: Optional[MotorWorker] = None
        self._jogging = False

        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(8)

        # ---- telemetry readouts
        tel = QtWidgets.QGridLayout()
        tel.setHorizontalSpacing(8)
        self.pos_labels = {}
        for col, (axis, key) in enumerate((("X", "x"), ("Y", "y"), ("Z", "z"))):
            cap = QtWidgets.QLabel(axis)
            cap.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: bold;")
            val = value_label()
            tel.addWidget(cap, 0, col)
            tel.addWidget(val, 1, col)
            self.pos_labels[key] = val
        unit = QtWidgets.QLabel("mm")
        unit.setStyleSheet(f"color: {theme.TEXT_DIM};")
        tel.addWidget(unit, 1, 3)

        self.driver_status = QtWidgets.QLabel("OFFLINE")
        self.driver_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.driver_status.setStyleSheet(
            f"background: {theme.FIELD}; border: 1px solid {theme.BORDER};"
            f" border-radius: 4px; padding: 4px; font-weight: bold; color: {theme.TEXT};"
        )
        tel.addWidget(self.driver_status, 0, 3, 2, 1)
        tel.setColumnStretch(3, 1)
        root.addLayout(tel)

        self.btn_enable = QtWidgets.QPushButton("Enable Drivers")
        self.btn_enable.setObjectName("primary")
        self.btn_enable.clicked.connect(self._toggle_enable)
        root.addWidget(self.btn_enable)

        # ---- feed / acceleration
        tune = QtWidgets.QGridLayout()
        tune.setHorizontalSpacing(8)
        tune.addWidget(QtWidgets.QLabel("Feed velocity"), 0, 0)
        self.spin_vel = QtWidgets.QDoubleSpinBox()
        self.spin_vel.setRange(VEL_MIN, VEL_MAX)
        self.spin_vel.setValue(25.0)
        self.spin_vel.setSingleStep(1.0)
        self.spin_vel.setSuffix(" mm/s")
        self.spin_vel.valueChanged.connect(lambda v: self.slider_vel.setValue(int(v)))
        self.slider_vel = QtWidgets.QSlider(Qt.Orientation.Horizontal)
        self.slider_vel.setRange(int(VEL_MIN), int(VEL_MAX))
        self.slider_vel.setValue(25)
        self.slider_vel.valueChanged.connect(lambda v: self.spin_vel.setValue(float(v)))

        tune.addWidget(QtWidgets.QLabel("Acceleration"), 1, 0)
        self.spin_accel = QtWidgets.QDoubleSpinBox()
        self.spin_accel.setRange(ACCEL_MIN, ACCEL_MAX)
        self.spin_accel.setValue(300.0)
        self.spin_accel.setSingleStep(10.0)
        self.spin_accel.setSuffix(" mm/s²")
        self.spin_accel.valueChanged.connect(lambda v: self.slider_accel.setValue(int(v)))
        self.slider_accel = QtWidgets.QSlider(Qt.Orientation.Horizontal)
        self.slider_accel.setRange(int(ACCEL_MIN), int(ACCEL_MAX))
        self.slider_accel.setValue(300)
        self.slider_accel.valueChanged.connect(lambda v: self.spin_accel.setValue(float(v)))

        tune.addWidget(self.spin_vel, 0, 1)
        tune.addWidget(self.slider_vel, 0, 2)
        tune.addWidget(self.spin_accel, 1, 1)
        tune.addWidget(self.slider_accel, 1, 2)
        tune.setColumnStretch(2, 1)
        root.addLayout(tune)

        # ---- jog pad
        # X/Y form a cross: X stacked vertically (X- above X+), Y side by side
        # horizontally (Y+ left of Y-), centre cell left empty.
        # Z is its own column of two opposite directions (Z- above Z+).
        jog = QtWidgets.QGridLayout()
        jog.setSpacing(5)
        self.jog_buttons = []
        spec = [
            ("X −", "x", -1, 0, 1),
            ("Y +", "y", +1, 1, 0),
            ("Y −", "y", -1, 1, 2),
            ("X +", "x", +1, 2, 1),
            ("Z −", "z", -1, 1, 4),
            ("Z +", "z", +1, 2, 4),
        ]
        for text, axis, sign, r, c in spec:
            btn = JogButton(text, axis, sign)
            btn.jog_pressed.connect(self._jog)
            btn.jog_released.connect(self._jog_release)
            jog.addWidget(btn, r, c)
            self.jog_buttons.append(btn)

        separator = QtWidgets.QFrame()
        separator.setFrameShape(QtWidgets.QFrame.Shape.VLine)
        separator.setFrameShadow(QtWidgets.QFrame.Shadow.Plain)
        separator.setStyleSheet(f"color: {theme.BORDER};")
        jog.addWidget(separator, 0, 3, 3, 1)

        self.btn_estop = QtWidgets.QPushButton("EMERGENCY STOP")
        self.btn_estop.setObjectName("danger")
        self.btn_estop.setMinimumHeight(38)
        self.btn_estop.clicked.connect(self._estop)
        jog.addWidget(self.btn_estop, 0, 5, 3, 1)
        jog.setColumnStretch(6, 1)
        root.addLayout(jog)

        self._set_motion_enabled(False)

    # -- binding ------------------------------------------------------------
    def bind(self, worker: Optional[MotorWorker]) -> None:
        self.worker = worker
        if worker is None:
            self._set_motion_enabled(False)
            self.driver_status.setText("OFFLINE")

    # -- telemetry ----------------------------------------------------------
    def update_telemetry(self, snap: MotorSnapshot) -> None:
        for key, lab in self.pos_labels.items():
            lab.setText(fmt(getattr(snap, key), "+7.2f", ""))
        if not snap.connected:
            text = "OFFLINE"
        elif snap.busy:
            text = "BUSY"
        elif not snap.enabled:
            text = "DISABLED"
        else:
            text = "IDLE / ENABLED"
        self.driver_status.setText(text)
        self.btn_enable.setText("Disable Drivers" if snap.enabled else "Enable Drivers")
        self.btn_enable.setEnabled(snap.connected)
        self._set_motion_enabled(snap.connected and snap.enabled)
        if snap.last_warning:
            self.driver_status.setToolTip(snap.last_warning)

    def _set_motion_enabled(self, on: bool) -> None:
        for btn in self.jog_buttons:
            btn.setEnabled(on)
            if not on:
                btn.force_release()
        self.slider_vel.setEnabled(on)
        self.slider_accel.setEnabled(on)
        self.spin_vel.setEnabled(on)
        self.spin_accel.setEnabled(on)
        self.btn_estop.setEnabled(on)

    # -- actions ------------------------------------------------------------
    def _toggle_enable(self) -> None:
        if self.worker is None:
            return
        currently = self.worker.snapshot().enabled
        self.worker.enable(not currently)

    def _jog(self, axis: str, sign: float) -> None:
        if self.worker is None:
            return
        snap = self.worker.snapshot()
        if not (snap.connected and snap.enabled):
            self._log("jog blocked: drivers are not enabled")
            return
        velocity = clamp(self.spin_vel.value(), VEL_MIN, VEL_MAX) * sign
        accel = clamp(self.spin_accel.value(), ACCEL_MIN, ACCEL_MAX)
        self._jogging = True
        self.worker.jog(axis, velocity, accel)

    def _jog_release(self) -> None:
        if self.worker is not None and self._jogging:
            self._jogging = False
            self.worker.jog_stop()

    def _estop(self) -> None:
        if self.worker is not None:
            self.worker.abort()
            self._log("EMERGENCY STOP issued")

    def release_jogs(self) -> None:
        """Force-stop any held jog (window deactivate / focus loss)."""
        any_down = False
        for btn in self.jog_buttons:
            if btn.is_down():
                any_down = True
            btn.force_release()
        if any_down or self._jogging:
            self._jogging = False
            if self.worker is not None:
                self.worker.jog_stop()


class GripperPanel(QtWidgets.QGroupBox):
    """STS3215 gripper telemetry and open/close control."""

    def __init__(self, log: Callable[[str], None], parent=None) -> None:
        super().__init__("Gripper  (STS3215 Servo)", parent)
        self._log = log
        self.worker: Optional[MotorWorker] = None
        self._state = "unknown"

        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(8)

        tel = QtWidgets.QGridLayout()
        tel.setHorizontalSpacing(8)
        self.vals = {}
        for col, (key, cap, spec, unit) in enumerate((
            ("pos", "Position", "6.0f", " step"),
            ("volt", "Supply", "5.1f", " V"),
            ("load", "Load", "5.1f", " %"),
            ("temp", "Temp", "4.0f", " °C"),
        )):
            tel.addWidget(QtWidgets.QLabel(cap), 0, col)
            lab = value_label()
            tel.addWidget(lab, 1, col)
            self.vals[key] = (lab, spec, unit)
        root.addLayout(tel)

        preset = QtWidgets.QHBoxLayout()
        self.btn_open = QtWidgets.QPushButton(f"Open  ({GRIPPER_OPEN})")
        self.btn_close = QtWidgets.QPushButton(f"Close ({GRIPPER_CLOSE})")
        self.btn_open.clicked.connect(lambda: self._preset(GRIPPER_OPEN, "open"))
        self.btn_close.clicked.connect(lambda: self._preset(GRIPPER_CLOSE, "close"))
        preset.addWidget(self.btn_open)
        preset.addWidget(self.btn_close)
        root.addLayout(preset)

        self.slider_pos = QtWidgets.QSlider(Qt.Orientation.Horizontal)
        self.slider_pos.setRange(GRIPPER_POS_MIN, GRIPPER_POS_MAX)
        self.slider_pos.setValue(GRIPPER_OPEN)
        self.spin_pos = QtWidgets.QSpinBox()
        self.spin_pos.setRange(GRIPPER_POS_MIN, GRIPPER_POS_MAX)
        self.spin_pos.setValue(GRIPPER_OPEN)
        self.slider_pos.valueChanged.connect(self.spin_pos.setValue)
        self.spin_pos.valueChanged.connect(self.slider_pos.setValue)

        posrow = QtWidgets.QHBoxLayout()
        posrow.addWidget(QtWidgets.QLabel("Target"))
        posrow.addWidget(self.slider_pos, 1)
        posrow.addWidget(self.spin_pos)
        self.btn_move = QtWidgets.QPushButton("Move")
        self.btn_move.clicked.connect(lambda: self._preset(self.spin_pos.value(), "custom"))
        posrow.addWidget(self.btn_move)
        root.addLayout(posrow)

        tune = QtWidgets.QHBoxLayout()
        tune.addWidget(QtWidgets.QLabel("Speed"))
        self.spin_speed = QtWidgets.QSpinBox()
        self.spin_speed.setRange(100, 3000)
        self.spin_speed.setValue(800)
        self.spin_speed.setSingleStep(50)
        self.spin_speed.setSuffix(" step/s")
        tune.addWidget(self.spin_speed)
        tune.addWidget(QtWidgets.QLabel("Accel"))
        self.spin_accel = QtWidgets.QSpinBox()
        self.spin_accel.setRange(10, 200)
        self.spin_accel.setValue(30)
        self.spin_accel.setSuffix(" ×100")
        tune.addWidget(self.spin_accel)
        tune.addStretch(1)
        root.addLayout(tune)

        self.state_label = QtWidgets.QLabel("state: unknown")
        self.state_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        root.addWidget(self.state_label)

        self._set_controls_enabled(False)

    # -- binding ------------------------------------------------------------
    def bind(self, worker: Optional[MotorWorker]) -> None:
        self.worker = worker
        if worker is None:
            self._set_controls_enabled(False)

    def _set_controls_enabled(self, on: bool) -> None:
        for w in (self.btn_open, self.btn_close, self.btn_move, self.spin_pos,
                  self.slider_pos, self.spin_speed, self.spin_accel):
            w.setEnabled(on)

    # -- telemetry ----------------------------------------------------------
    def update_telemetry(self, snap: MotorSnapshot) -> None:
        mapping = {"pos": snap.servo_pos, "volt": snap.servo_volt_v,
                   "load": snap.servo_load_pct, "temp": float(snap.servo_temp_c)}
        for key, (lab, spec, unit) in self.vals.items():
            lab.setText(fmt(mapping[key], spec, unit))
        self._set_controls_enabled(snap.connected)
        self.state_label.setText(
            f"state: {self._state}   |   servo pos {snap.servo_pos} step"
            + ("   |   moving" if snap.servo_moving else "")
        )

    # -- actions ------------------------------------------------------------
    def _preset(self, pos: int, state: str) -> None:
        if self.worker is None:
            return
        pos = int(clamp(pos, GRIPPER_POS_MIN, GRIPPER_POS_MAX))
        speed = self.spin_speed.value()
        accel = self.spin_accel.value()
        self.spin_pos.blockSignals(True)
        self.spin_pos.setValue(pos)
        self.spin_pos.blockSignals(False)
        self.slider_pos.blockSignals(True)
        self.slider_pos.setValue(pos)
        self.slider_pos.blockSignals(False)
        self._state = state
        self.worker.servo_move(pos, speed, accel)
        self._log(f"gripper -> {state} ({pos}) speed={speed} accel={accel}")

    def set_state(self, state: str) -> None:
        self._state = state
