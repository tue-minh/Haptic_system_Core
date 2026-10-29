"""Connection manager for the COM ports (Phase 2).

Roles
-----
``motor``     STM32 master board: CoreXY + Z steppers, the STS3215 gripper, and
              the 100 Hz haptic output frames - **all on this one USB CDC port**.
``sensor_l``  Left PaXini tactile sensor board (its own port).
``sensor_r``  Right PaXini tactile sensor board (its own port).

That is three ports in total.  The haptic stream shares the motor/STM32 link
rather than owning a port (see ``unified_gui.haptic.Stm32HapticStreamer``'s
``writer`` argument), so it has no row of its own here.

The panel only *collects* the user's choices and emits request signals; the
main window owns the actual workers so worker lifetime and shutdown stay in a
single place.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from PyQt6 import QtWidgets
from PyQt6.QtCore import pyqtSignal

from .. import theme
from .. import _paths  # noqa: F401
from ..sensors import SENSOR_BAUDRATE
from ..widgets import StatusLed

import serial.tools.list_ports

# Line rates offered per role. The sensor default is taken straight from
# paxini_usb_core so it always matches the reference reader; the STM32 board is
# 1 Mbaud per the project spec. All stay editable.
BAUD_CHOICES = (921600, 1000000, 500000, 250000, 115200)

ROLES = ("motor", "sensor_l", "sensor_r")
ROLE_LABELS = {
    "motor": "STM32 (motor + haptic)",
    "sensor_l": "Left Sensor",
    "sensor_r": "Right Sensor",
}
DEFAULT_BAUD = {
    "motor": 1000000,
    "sensor_l": SENSOR_BAUDRATE,
    "sensor_r": SENSOR_BAUDRATE,
}


@dataclass
class _RoleWidgets:
    combo: QtWidgets.QComboBox
    baud: QtWidgets.QComboBox
    button: QtWidgets.QPushButton
    led: StatusLed
    status: QtWidgets.QLabel


class ConnectionPanel(QtWidgets.QGroupBox):
    connect_requested = pyqtSignal(str)
    disconnect_requested = pyqtSignal(str)
    refresh_requested = pyqtSignal()
    connect_all_requested = pyqtSignal()
    disconnect_all_requested = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__("Connection", parent)
        self.rows: Dict[str, _RoleWidgets] = {}
        self._ports = []
        self._connected: set = set()

        outer = QtWidgets.QVBoxLayout(self)
        outer.setSpacing(6)

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)
        for col, head in enumerate(("Role", "COM port", "Baud", "Link", "")):
            grid.addWidget(QtWidgets.QLabel(head), 0, col)

        for i, role in enumerate(ROLES, start=1):
            combo = QtWidgets.QComboBox()
            combo.setMinimumWidth(130)
            baud = QtWidgets.QComboBox()
            baud.addItems([str(b) for b in BAUD_CHOICES])
            baud.setCurrentText(str(DEFAULT_BAUD[role]))
            baud.setFixedWidth(78)
            btn = QtWidgets.QPushButton("Connect")
            btn.setFixedWidth(86)
            btn.clicked.connect(lambda _c=False, r=role: self._on_button(r))
            led = StatusLed("off")
            status = QtWidgets.QLabel("-")
            status.setStyleSheet(f"color: {theme.TEXT_DIM};")
            # The link text can be long; let it clip rather than dictate the
            # column's minimum width (the full text stays available as a tooltip).
            status.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                                 QtWidgets.QSizePolicy.Policy.Preferred)
            status.setMinimumWidth(70)

            grid.addWidget(QtWidgets.QLabel(ROLE_LABELS[role]), i, 0)
            grid.addWidget(combo, i, 1)
            grid.addWidget(baud, i, 2)
            grid.addWidget(btn, i, 3)
            grid.addWidget(led, i, 4)
            grid.addWidget(status, i, 5)
            grid.setColumnStretch(5, 1)

            self.rows[role] = _RoleWidgets(combo, baud, btn, led, status)

        outer.addLayout(grid)

        actions = QtWidgets.QHBoxLayout()
        self.btn_refresh = QtWidgets.QPushButton("Refresh ports")
        self.btn_refresh.clicked.connect(self.refresh_requested.emit)
        self.btn_connect_all = QtWidgets.QPushButton("Connect All")
        self.btn_connect_all.setObjectName("primary")
        self.btn_connect_all.clicked.connect(self.connect_all_requested.emit)
        self.btn_disconnect_all = QtWidgets.QPushButton("Disconnect All")
        self.btn_disconnect_all.clicked.connect(self.disconnect_all_requested.emit)
        actions.addWidget(self.btn_refresh)
        actions.addStretch(1)
        actions.addWidget(self.btn_connect_all)
        actions.addWidget(self.btn_disconnect_all)
        outer.addLayout(actions)

    # -- port enumeration (Phase 2.1) ---------------------------------------
    def scan_ports(self, auto_assign: bool = False) -> List:
        self._ports = list(serial.tools.list_ports.comports())
        for role in ROLES:
            self._fill_combo(self.rows[role].combo)
        if auto_assign:
            self._auto_assign()
        return self._ports

    def _auto_assign(self) -> None:
        """Spread the detected ports across the roles in order.

        Mirrors the reference reader's ``refresh_ports(auto_assign=True)`` so a
        typical setup needs no manual selection; the user can still change any
        combo afterwards or hit Refresh ports.
        """
        devices = [p.device for p in self._ports]
        for role, device in zip(ROLES, devices):
            if role in self._connected:
                continue
            combo = self.rows[role].combo
            idx = combo.findData(device)
            if idx >= 0:
                combo.setCurrentIndex(idx)

    def port_devices(self) -> List[str]:
        return [p.device for p in self._ports]

    @staticmethod
    def _label(port) -> str:
        desc = getattr(port, "description", "") or ""
        return f"{port.device} - {desc}" if desc else port.device

    def _fill_combo(self, combo: QtWidgets.QComboBox) -> None:
        current = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        for port in self._ports:
            combo.addItem(self._label(port), port.device)
        if current:
            idx = combo.findData(current)
            if idx >= 0:
                combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    # -- accessors ----------------------------------------------------------
    def selected_port(self, role: str) -> str:
        combo = self.rows[role].combo
        return combo.currentData() or combo.currentText().split(" ")[0]

    def selected_baud(self, role: str) -> int:
        try:
            return int(self.rows[role].baud.currentText())
        except ValueError:
            return DEFAULT_BAUD[role]

    def collision(self, role: str) -> Optional[str]:
        """Return the connected role already holding ``role``'s port, if any."""
        port = self.selected_port(role)
        if not port:
            return None
        for other in ROLES:
            if other != role and other in self._connected and self.selected_port(other) == port:
                return other
        return None

    # -- UI state -----------------------------------------------------------
    def _on_button(self, role: str) -> None:
        if role in self._connected:
            self.disconnect_requested.emit(role)
            return
        conflict = self.collision(role)
        if conflict:
            QtWidgets.QMessageBox.warning(
                self, "Port collision",
                f"{self.selected_port(role)} is already used by "
                f"{ROLE_LABELS[conflict]}.\nPick another port or disconnect "
                "that role first.",
            )
            return
        self.connect_requested.emit(role)

    def set_state(self, role: str, state: str, text: str = "") -> None:
        w = self.rows[role]
        w.led.set_state(state)
        w.status.setText(text)
        w.status.setToolTip(f"{ROLE_LABELS[role]}: {text}" if text else "")
        connected = state in ("connecting", "ready", "error")
        if connected:
            self._connected.add(role)
        else:
            self._connected.discard(role)
        w.combo.setEnabled(not connected)
        w.baud.setEnabled(not connected)
        w.button.setText("Disconnect" if connected else "Connect")

    def is_connected(self, role: str) -> bool:
        return role in self._connected

    def lock_all(self, locked: bool) -> None:
        """Freeze every control (used while the app is tearing down)."""
        for role in ROLES:
            w = self.rows[role]
            w.button.setEnabled(not locked)
            w.baud.setEnabled(not locked)
            w.combo.setEnabled(not locked and not self.is_connected(role))
