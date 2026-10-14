"""Tactile sensor monitoring panel (Phase 4.1, 4.2, 4.3, 4.6).

* real-time 2D scrolling curves (pyqtgraph) for both fingers;
* direction filter (Fz / Fx / Fy / All) that toggles curve visibility without
  discarding the history buffer;
* software tare buttons (left / right / both / reset);
* live numerical metrics grid (normal, shear, magnitude, mu, angles, contact).
"""
from __future__ import annotations

from typing import Callable, Dict, Optional

import numpy as np
from PyQt6 import QtWidgets
from PyQt6.QtCore import Qt
import pyqtgraph as pg

from .. import theme
from .. import _paths  # noqa: F401
import paxini_usb_core as core
from ..sensors import SensorLink
from ..widgets import fmt

# display name -> (metric key, format spec)
METRIC_ROWS = [
    ("Fz  normal (N)", "fz", "7.2f"),
    ("Ft  shear (N)", "f_shear", "7.2f"),
    ("|F| total (N)", "f_mag", "7.2f"),
    ("\u03bc = Ft/Fz", "mu", "7.3f"),
    ("tilt \u03c8 (deg)", "tilt", "7.1f"),
    ("shear dir \u03b8 (deg)", "shear_dir", "+7.1f"),
    ("contact taxels", "contact_taxels", "5.0f"),
    ("peak taxel #", "peak_taxel", "5.0f"),
    ("peak Fz (N)", "peak_fz", "7.2f"),
]

# direction -> (label, metric key, ring column, pen style, checked by default)
DIRECTIONS = [
    ("Fz", "fz", core.COL["Fz"], Qt.PenStyle.SolidLine, True),
    ("Fx", "fx", core.COL["Fx"], Qt.PenStyle.DashLine, False),
    ("Fy", "fy", core.COL["Fy"], Qt.PenStyle.DotLine, False),
    ("All forces", "all", None, Qt.PenStyle.SolidLine, False),
]
DIRECTION_TOOLTIPS = {
    0: "Fz - normal force (default)",
    1: "Fx - lateral shear force",
    2: "Fy - longitudinal shear force",
    3: "Show Fx, Fy and Fz together",
}
# Sensor identity is carried by lightness (light = Left, dark = Right); the pen
# style carries the force direction. No hue is used.
TAG_COLOR = theme.SENSOR_SHADE
TAG_LABEL = {"L": "Left", "R": "Right"}


class SensorMonitorPanel(QtWidgets.QWidget):
    def __init__(self, log: Callable[[str], None], parent=None) -> None:
        super().__init__(parent)
        self._log = log
        self.links: Dict[str, SensorLink] = {}
        self.curves: Dict[tuple, pg.PlotDataItem] = {}
        self._window_s = 10.0

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(6)

        # ---- toolbar, split over two rows so the panel can stay narrow
        tools = QtWidgets.QVBoxLayout()
        tools.setSpacing(4)

        row_dir = QtWidgets.QHBoxLayout()
        row_dir.addWidget(QtWidgets.QLabel("Direction:"))
        self.dir_group = QtWidgets.QButtonGroup(self)
        for i, (name, _key, _col, _style, checked) in enumerate(DIRECTIONS):
            rb = QtWidgets.QRadioButton(name)
            rb.setChecked(checked)
            rb.setToolTip(DIRECTION_TOOLTIPS[i])
            rb.toggled.connect(lambda on: on and self._apply_direction())
            self.dir_group.addButton(rb, i)
            row_dir.addWidget(rb)
        row_dir.addSpacing(10)

        self.chk_mag = QtWidgets.QCheckBox("|F|")
        self.chk_shear = QtWidgets.QCheckBox("Ft")
        self.chk_mag.setToolTip("Overlay total magnitude |F|")
        self.chk_shear.setToolTip("Overlay tangential shear Ft")
        self.chk_mag.toggled.connect(self._apply_direction)
        self.chk_shear.toggled.connect(self._apply_direction)
        row_dir.addWidget(self.chk_mag)
        row_dir.addWidget(self.chk_shear)
        row_dir.addStretch(1)
        tools.addLayout(row_dir)

        row_ctl = QtWidgets.QHBoxLayout()
        self.btn_tare_l = QtWidgets.QPushButton("Tare L")
        self.btn_tare_r = QtWidgets.QPushButton("Tare R")
        self.btn_tare_b = QtWidgets.QPushButton("Tare Both")
        self.btn_tare_reset = QtWidgets.QPushButton("Reset Tare")
        self.btn_tare_l.setToolTip("Zero the Left sensor (average the next 50 frames)")
        self.btn_tare_r.setToolTip("Zero the Right sensor (average the next 50 frames)")
        self.btn_tare_b.setToolTip("Zero both sensors")
        self.btn_tare_reset.setToolTip("Restore the raw, un-tared baseline")
        self.btn_tare_l.clicked.connect(lambda: self._tare("L"))
        self.btn_tare_r.clicked.connect(lambda: self._tare("R"))
        self.btn_tare_b.clicked.connect(lambda: self._tare("B"))
        self.btn_tare_reset.clicked.connect(self._reset_tare)
        for b in (self.btn_tare_l, self.btn_tare_r, self.btn_tare_b, self.btn_tare_reset):
            row_ctl.addWidget(b)

        row_ctl.addStretch(1)
        row_ctl.addWidget(QtWidgets.QLabel("Window"))
        self.spin_window = QtWidgets.QDoubleSpinBox()
        self.spin_window.setRange(1.0, 60.0)
        self.spin_window.setValue(self._window_s)
        self.spin_window.setSuffix(" s")
        self.spin_window.setFixedWidth(72)
        self.spin_window.setToolTip("Visible time window of the scrolling plot")
        row_ctl.addWidget(self.spin_window)
        self.chk_pause = QtWidgets.QCheckBox("Pause")
        row_ctl.addWidget(self.chk_pause)
        tools.addLayout(row_ctl)

        root.addLayout(tools)

        # ---- plot + metrics side by side
        split = QtWidgets.QSplitter(Qt.Orientation.Horizontal)

        self.plot = pg.PlotWidget()
        pi = self.plot.getPlotItem()
        pi.setTitle("Resultant force history", color=theme.TEXT, size="10pt")
        pi.showGrid(x=True, y=True, alpha=0.25)
        pi.addLegend(offset=(6, 6), labelTextSize="8pt")
        pi.setDownsampling(auto=True, mode="peak")
        pi.setClipToView(True)
        pi.enableAutoRange(axis="x", enable=False)
        pi.setLabel("bottom", "time", units="s")
        pi.setLabel("left", "force", units="N")
        split.addWidget(self.plot)

        self._build_metrics()
        split.addWidget(self.metrics_box)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        root.addWidget(split, 1)

        self._rebuild_curves()
        self._apply_direction()

    # -- metrics grid -------------------------------------------------------
    def _build_metrics(self) -> None:
        self.metrics_box = QtWidgets.QGroupBox("Live metrics")
        grid = QtWidgets.QGridLayout(self.metrics_box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(3)

        grid.addWidget(_col_header("Metric"), 0, 0)
        grid.addWidget(_col_header("Left"), 0, 1)
        grid.addWidget(_col_header("Right"), 0, 2)

        self.metric_labels: Dict[tuple, QtWidgets.QLabel] = {}
        for r, (name, key, spec) in enumerate(METRIC_ROWS, start=1):
            grid.addWidget(QtWidgets.QLabel(name), r, 0)
            for c, tag in ((1, "L"), (2, "R")):
                lab = QtWidgets.QLabel("---")
                lab.setFont(QtWidgets.QApplication.font())
                lab.setStyleSheet(theme.VALUE_QSS)
                lab.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                grid.addWidget(lab, r, c)
                self.metric_labels[(tag, key)] = lab
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)

    # -- links --------------------------------------------------------------
    def bind(self, links: Dict[str, SensorLink]) -> None:
        self.links = dict(links)
        self._rebuild_curves()
        self._apply_direction()

    # -- curves -------------------------------------------------------------
    def _rebuild_curves(self) -> None:
        pi = self.plot.getPlotItem()
        for curve in self.curves.values():
            pi.removeItem(curve)
        self.curves.clear()

        for tag in ("L", "R"):
            if tag not in self.links:
                continue
            color = TAG_COLOR[tag]
            for name, key, _col, style, _vis in DIRECTIONS:
                if key == "all":
                    continue
                pi_curve = pg.PlotDataItem(
                    pen=pg.mkPen(color, width=1.7, style=style),
                    name=f"{TAG_LABEL[tag]} {key.upper()}", connect="finite",
                )
                pi.addItem(pi_curve)
                self.curves[(tag, key)] = pi_curve
            # overlays get their own pen patterns so they stay separable
            for key, style in (("f_mag", Qt.PenStyle.DashDotLine),
                              ("f_shear", Qt.PenStyle.DashDotDotLine)):
                pi_curve = pg.PlotDataItem(
                    pen=pg.mkPen(color, width=1.5, style=style),
                    name=f"{TAG_LABEL[tag]} {key}", connect="finite",
                )
                pi.addItem(pi_curve)
                self.curves[(tag, key)] = pi_curve

    def _apply_direction(self) -> None:
        idx = self.dir_group.checkedId()
        _name, key, _col, _style, _vis = DIRECTIONS[max(idx, 0)]
        wanted = set()
        if key == "all":
            wanted.update({"fx", "fy", "fz"})
        else:
            wanted.add(key)
        if self.chk_mag.isChecked():
            wanted.add("f_mag")
        if self.chk_shear.isChecked():
            wanted.add("f_shear")
        for (tag, ckey), curve in self.curves.items():
            curve.setVisible(ckey in wanted)
        self._current_dirs = wanted

    def selected_direction(self) -> str:
        idx = max(self.dir_group.checkedId(), 0)
        return DIRECTIONS[idx][0]

    # -- tare ---------------------------------------------------------------
    def _tare(self, which: str) -> None:
        targets = ("L", "R") if which == "B" else (which,)
        for tag in targets:
            link = self.links.get(tag)
            if link is not None:
                link.request_tare(50)
        self._log(f"tare requested: {which}")

    def _reset_tare(self) -> None:
        for link in self.links.values():
            link.reset_tare()
        self._log("tare reset")

    # -- periodic refresh ---------------------------------------------------
    def refresh(self, now: float, fps_window: Optional[float] = None) -> None:
        window = fps_window if fps_window is not None else self.spin_window.value()
        t_from = now - window

        key_to_col = {"fx": core.COL["Fx"], "fy": core.COL["Fy"], "fz": core.COL["Fz"],
                      "f_mag": core.COL["F_mag"], "f_shear": core.COL["F_shear"]}
        cols = [key_to_col[k] for k in key_to_col]

        for tag, link in self.links.items():
            if not self.chk_pause.isChecked():
                data = link.ring_window(t_from, [0] + cols)
                if data.shape[0]:
                    x = data[:, 0]
                    for j, k in enumerate(key_to_col):
                        curve = self.curves.get((tag, k))
                        if curve is not None and curve.isVisible():
                            curve.setData(x, data[:, j + 1])
            reading = link.reading()
            for _name, key, spec in METRIC_ROWS:
                lab = self.metric_labels.get((tag, key))
                if lab is None:
                    continue
                lab.setText(fmt(getattr(reading, key) if reading else None, spec))

        if not self.chk_pause.isChecked():
            self.plot.setXRange(t_from, now, padding=0)


def _col_header(text: str) -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    lab.setStyleSheet(f"color: {theme.TEXT_DIM}; font-weight: bold;")
    return lab
