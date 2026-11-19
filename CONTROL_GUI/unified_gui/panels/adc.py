"""ITV valve ADC feedback panel (STM32F103 slave over SPI).

The master board polls the STM32F103 ADC slave in the background (CAN
0x500-0x503) and caches the five 12-bit channels; this panel displays the
snapshot the motor worker already fetches and lets the user change the
master's polling slot/rate via CAN 0x502.
"""
from __future__ import annotations

from typing import Dict

from PyQt6 import QtWidgets
from PyQt6.QtCore import pyqtSignal

from .. import theme
from ..widgets import value_label

CHANNELS = (
    ("CH0", "PA0"),
    ("CH1", "PA1"),
    ("CH2", "PA2"),
    ("CH3", "PA3"),
    ("CH4", "PA4"),
)
PERIOD_CHOICES = (10, 20, 50, 100)
VREF_MV = 3300.0
ADC_MAX = 4095


class AdcPanel(QtWidgets.QGroupBox):
    apply_requested = pyqtSignal(bool, int, int, int)
    read_requested = pyqtSignal()

    def __init__(self, parent=None) -> None:
        super().__init__("ITV valve ADC  (STM32F103 SPI slave)", parent)

        self._config_seeded = False
        self.raw_labels: Dict[int, QtWidgets.QLabel] = {}
        self.mv_labels: Dict[int, QtWidgets.QLabel] = {}
        self.bars: Dict[int, QtWidgets.QProgressBar] = {}

        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(6)

        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(3)
        grid.addWidget(_header("Channel"), 0, 0)
        grid.addWidget(_header("Raw"), 0, 1)
        grid.addWidget(_header("Millivolts"), 0, 2)
        grid.addWidget(_header("Level"), 0, 3)

        for row, (name, pin) in enumerate(CHANNELS, start=1):
            lab = QtWidgets.QLabel(f"{name} · {pin}")
            grid.addWidget(lab, row, 0)

            raw = value_label()
            raw.setText("---")
            grid.addWidget(raw, row, 1)
            self.raw_labels[row - 1] = raw

            mv = value_label()
            mv.setText("---")
            grid.addWidget(mv, row, 2)
            self.mv_labels[row - 1] = mv

            bar = QtWidgets.QProgressBar()
            bar.setRange(0, ADC_MAX)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFixedHeight(10)
            bar.setMinimumWidth(90)
            bar.setStyleSheet(
                f"QProgressBar {{ background: {theme.PANEL_ALT};"
                f" border: 1px solid {theme.BORDER}; border-radius: 3px; }}"
                f" QProgressBar::chunk {{ background: {theme.PRIMARY};"
                " border-radius: 2px; }}"
            )
            grid.addWidget(bar, row, 3)
            self.bars[row - 1] = bar

        grid.setColumnStretch(3, 1)
        root.addLayout(grid)

        self.lbl_status = QtWidgets.QLabel("no data")
        self.lbl_status.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: Consolas, monospace;")
        self.lbl_status.setWordWrap(True)
        root.addWidget(self.lbl_status)

        controls = QtWidgets.QHBoxLayout()
        self.chk_enable = QtWidgets.QCheckBox("Poll")
        self.chk_enable.setChecked(True)
        self.chk_enable.setToolTip("Enable the master's background SPI ADC polling")
        controls.addWidget(self.chk_enable)

        controls.addWidget(QtWidgets.QLabel("Bus"))
        self.cbo_bus = QtWidgets.QComboBox()
        self.cbo_bus.addItem("SPI1", 1)
        self.cbo_bus.addItem("SPI3", 3)
        self.cbo_bus.setFixedWidth(64)
        controls.addWidget(self.cbo_bus)

        controls.addWidget(QtWidgets.QLabel("CS"))
        self.cbo_cs = QtWidgets.QComboBox()
        for idx in range(3):
            self.cbo_cs.addItem(f"CS{idx + 1}", idx)
        self.cbo_cs.setFixedWidth(58)
        controls.addWidget(self.cbo_cs)

        controls.addWidget(QtWidgets.QLabel("Period"))
        self.cbo_period = QtWidgets.QComboBox()
        for ms in PERIOD_CHOICES:
            self.cbo_period.addItem(f"{ms} ms", ms)
        self.cbo_period.setFixedWidth(72)
        controls.addWidget(self.cbo_period)

        self.btn_apply = QtWidgets.QPushButton("Apply")
        self.btn_apply.setToolTip("Send CAN 0x502 to change the master polling slot/rate")
        self.btn_apply.clicked.connect(self._emit_apply)
        controls.addWidget(self.btn_apply)

        self.btn_read = QtWidgets.QPushButton("Read now")
        self.btn_read.clicked.connect(self.read_requested.emit)
        controls.addWidget(self.btn_read)
        controls.addStretch(1)
        root.addLayout(controls)

        self.lbl_footer = QtWidgets.QLabel("not connected")
        self.lbl_footer.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.lbl_footer.setWordWrap(True)
        root.addWidget(self.lbl_footer)

        self.set_available(False)

    # -- state --------------------------------------------------------------
    def set_available(self, available: bool, detail: str = "") -> None:
        self.btn_apply.setEnabled(available)
        self.btn_read.setEnabled(available)
        if available:
            self.lbl_footer.setText("streaming from the STM32 SPI link")
            return

        self._config_seeded = False
        self.lbl_footer.setText(detail or "not connected")
        self.lbl_status.setText("no data")
        self.lbl_status.setStyleSheet(f"color: {theme.TEXT_DIM};")
        for i in range(len(CHANNELS)):
            self.raw_labels[i].setText("---")
            self.mv_labels[i].setText("---")
            self.bars[i].setValue(0)

    def clear(self) -> None:
        self.set_available(False, "not connected")

    def update_telemetry(self, snap) -> None:
        if (snap is None or not getattr(snap, "connected", False)
                or not getattr(snap, "adc_available", False)):
            detail = getattr(snap, "adc_last_error", "") if snap is not None else ""
            self.set_available(False, detail)
            return

        self.set_available(True)
        channels = tuple(int(v) for v in snap.adc_channels)
        for i, raw in enumerate(channels):
            clipped = max(0, min(ADC_MAX, raw))
            self.raw_labels[i].setText(f"{clipped:4d}")
            self.mv_labels[i].setText(f"{clipped * VREF_MV / ADC_MAX:7.1f} mV")
            self.bars[i].setValue(clipped)

        flags = ["valid" if snap.adc_valid else "no-valid"]
        if snap.adc_error:
            flags.append("ADC-ERR")
        color = theme.DANGER if snap.adc_error else theme.TEXT_DIM
        self.lbl_status.setStyleSheet(
            f"color: {color}; font-family: Consolas, monospace;")
        self.lbl_status.setText(
            f"seq {snap.adc_seq:3d} · status 0x{snap.adc_status:02X} "
            f"[{', '.join(flags)}] · ok {snap.adc_rx_ok} · "
            f"crc-err {snap.adc_crc_err} · {snap.adc_hz:4.1f} Hz")

        if not self._config_seeded:
            self._seed_config(snap)

    def _seed_config(self, snap) -> None:
        _select_data(self.cbo_bus, int(snap.adc_bus_id), "SPI{}")
        _select_data(self.cbo_cs, int(snap.adc_cs_idx), "CS{}")
        _select_data(self.cbo_period, int(snap.adc_period_ms), "{} ms")
        self.chk_enable.setChecked(bool(snap.adc_enabled))
        self._config_seeded = True

    def _emit_apply(self) -> None:
        self.apply_requested.emit(
            self.chk_enable.isChecked(),
            int(self.cbo_bus.currentData()),
            int(self.cbo_cs.currentData()),
            int(self.cbo_period.currentData()),
        )


def _header(text: str) -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    lab.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: bold;")
    return lab


def _select_data(combo: QtWidgets.QComboBox, value: int, label_fmt: str) -> None:
    idx = combo.findData(value)
    if idx < 0:
        combo.addItem(label_fmt.format(value), value)
        idx = combo.findData(value)
    combo.setCurrentIndex(idx)
