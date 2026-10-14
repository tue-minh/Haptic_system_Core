"""Small shared widgets / formatting helpers used across the panels."""
from __future__ import annotations

import math
from typing import Optional

from PyQt6 import QtWidgets
from PyQt6.QtCore import Qt

from . import theme

LED_COLORS = {
    "off": theme.LED_OFF,
    "connecting": theme.LED_CONNECTING,
    "ready": theme.LED_READY,
    "error": theme.LED_ERROR,
}


class StatusLed(QtWidgets.QLabel):
    """Round coloured connection indicator.

    States: ``off`` (red), ``connecting`` (yellow), ``ready`` (green),
    ``error`` (orange).
    """

    def __init__(self, state: str = "off", parent=None) -> None:
        super().__init__(parent)
        self.setFixedSize(16, 16)
        self.set_state(state)

    def set_state(self, state: str) -> None:
        color = LED_COLORS.get(state, theme.LED_OFF)
        self.setStyleSheet(theme.led_qss(color))
        self.setToolTip(f"status: {state}")


def fmt(value: Optional[float], spec: str = "7.2f", unit: str = "", na: str = "  ---") -> str:
    """Format a possibly-NaN/None number for a monospace readout."""
    if value is None:
        return na
    try:
        v = float(value)
    except (TypeError, ValueError):
        return na
    if math.isnan(v) or math.isinf(v):
        return na
    return f"{v:{spec}}{unit}"


def value_label() -> QtWidgets.QLabel:
    """A fixed-width monospace readout label."""
    lab = QtWidgets.QLabel()
    lab.setFont(theme_mono_font())
    lab.setStyleSheet(theme.VALUE_QSS)
    lab.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    lab.setMinimumWidth(84)
    return lab


def theme_mono_font():
    from PyQt6 import QtGui

    return QtGui.QFont("Consolas", 9)
