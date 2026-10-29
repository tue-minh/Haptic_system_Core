"""Haptic output panel.

The 100 Hz haptic stream rides the STM32 link that already carries motor
control, so it needs no COM port of its own - just an on/off switch and a status
line.  The five valves themselves are visualised by ``unified_gui.valvemap``.

Streaming starts automatically once the STM32 link is ready (if enabled) and
stops when the link drops, so this panel never has to be touched for the normal
connect-flow.
"""
from __future__ import annotations

from typing import Optional

from PyQt6 import QtWidgets
from PyQt6.QtCore import pyqtSignal

from .. import theme
from ..haptic import HAPTIC_RATE_HZ, CAN_ID, HapticSnapshot


class HapticPanel(QtWidgets.QGroupBox):
    enabled_changed = pyqtSignal(bool)
    ping_requested = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__("Haptic output  (shares the STM32 link)", parent)

        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(6)

        self.chk_enable = QtWidgets.QCheckBox("Stream haptic frames")
        self.chk_enable.setChecked(True)
        self.chk_enable.setToolTip(
            f"Sends the 5-valve payload to the STM32 at {HAPTIC_RATE_HZ:.0f} Hz "
            f"(CAN 0x{CAN_ID:03X}) on the same COM port as motor control.")
        self.chk_enable.toggled.connect(self.enabled_changed.emit)
        root.addWidget(self.chk_enable)

        self.lbl_state = QtWidgets.QLabel("waiting for the STM32 link")
        self.lbl_state.setStyleSheet(theme.VALUE_QSS)
        self.lbl_state.setWordWrap(True)
        root.addWidget(self.lbl_state)

        self.lbl_detail = QtWidgets.QLabel(
            f"CAN 0x{CAN_ID:03X} · 8-byte payload → valves A-E")
        self.lbl_detail.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.lbl_detail.setWordWrap(True)
        root.addWidget(self.lbl_detail)

        self.btn_ping = QtWidgets.QPushButton("Ping ESP32 (UART 250k)")
        self.btn_ping.setToolTip(
            "Sends CAN 0x301 to the STM32, which pings the ESP32 slave over "
            "USART2 at 250 kbaud and relays its status.")
        self.btn_ping.clicked.connect(self.ping_requested.emit)
        self.btn_ping.setEnabled(False)
        root.addWidget(self.btn_ping)

        self.lbl_ping = QtWidgets.QLabel("ESP32 ping: not run")
        self.lbl_ping.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.lbl_ping.setWordWrap(True)
        root.addWidget(self.lbl_ping)
        self._last_ping_text = ""

    # -- state --------------------------------------------------------------
    @property
    def enabled(self) -> bool:
        return self.chk_enable.isChecked()

    def set_ping_result(self, text: str, ok: Optional[bool]) -> None:
        """Show the latest ESP32 UART ping result (``None`` = in progress)."""
        if not text or text == self._last_ping_text:
            return
        self._last_ping_text = text
        if ok is None:
            color = theme.TEXT_DIM
        elif ok:
            color = theme.LED_READY
        else:
            color = theme.DANGER
        self.lbl_ping.setStyleSheet(f"color: {color};")
        self.lbl_ping.setText(text)

    def update_status(
        self,
        snap: Optional[HapticSnapshot],
        streaming: bool,
        motor_connected: bool,
    ) -> None:
        self.btn_ping.setEnabled(bool(motor_connected))

        if not self.enabled:
            self.lbl_state.setText("off")
            self.lbl_detail.setText("tick 'Stream haptic frames' to send to the STM32")
            return

        if not motor_connected:
            self.lbl_state.setText("waiting for the STM32 link")
            self.lbl_detail.setText(
                f"CAN 0x{CAN_ID:03X} · starts automatically once the motor port is up")
            return

        if not streaming or snap is None:
            self.lbl_state.setText("starting…")
            return

        if snap.error:
            self.lbl_state.setText(f"error: {snap.error}")
        else:
            self.lbl_state.setText(
                f"streaming · {snap.rate_hz:4.1f} Hz · {snap.sent} frames")

        flags = ", ".join(snap.flag_names) or "none"
        self.lbl_detail.setText(
            f"CAN 0x{CAN_ID:03X} · flags 0x{snap.flags:02X} [{flags}] · seq {snap.seq}")
