"""Live 5-valve haptic output map.

The tactile taxel map's counterpart for the *output* side: it shows exactly the
five pneumatic chambers the slave thimble drives, at their physical places on
the operator's finger, coloured by the force the STM32 master is being told to
render.  Where the taxel map plots 52 sensor points, this plots the 5 actuator
points that the synthesised 8-byte payload maps onto:

* **A / B / C** - underside pad, Tip -> Mid -> Base (chambers 1..3).
* **D / E**     - the two flanks (chambers 4 and 5).

Each point carries its position (fixed by :data:`unified_gui.haptic.VALVE_LAYOUT`),
the force the frame commands (N), the raw payload byte that drives it, and how
long ago ("when the data came") the driving frame was emitted.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from PyQt6 import QtWidgets
from PyQt6.QtCore import Qt
import pyqtgraph as pg

from . import theme
from .haptic import VALVE_LAYOUT, MAX_FORCE_N, HapticSnapshot
from .taxelmap import _lookup_table
from .widgets import fmt

VALVE_SIZE_MIN, VALVE_SIZE_MAX = 16.0, 40.0
FULL_SCALE_MIN, FULL_SCALE_MAX = 0.5, MAX_FORCE_N
CMAPS = ("Greys", "cividis", "viridis", "inferno")
POSITION_PAD = 8.0


class ValveMapView(QtWidgets.QWidget):
    """The five output valves: position, commanded force and frame freshness."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._full_scale = FULL_SCALE_MAX
        self.lut = _lookup_table(theme.TAXEL_CMAP)
        self._brush_key: Optional[bytes] = None

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(4)

        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(6)
        self.spin_fs = QtWidgets.QDoubleSpinBox()
        self.spin_fs.setRange(FULL_SCALE_MIN, FULL_SCALE_MAX)
        self.spin_fs.setSingleStep(1.0)
        self.spin_fs.setValue(self._full_scale)
        self.spin_fs.setSuffix(" N")
        self.spin_fs.setFixedWidth(80)
        self.spin_fs.setToolTip("Valve colour / size full-scale force")
        self.spin_fs.valueChanged.connect(self._set_full_scale)

        self.combo_cmap = QtWidgets.QComboBox()
        self.combo_cmap.addItems(CMAPS)
        self.combo_cmap.setCurrentText(theme.TAXEL_CMAP if theme.TAXEL_CMAP in CMAPS else CMAPS[0])
        self.combo_cmap.setFixedWidth(96)
        self.combo_cmap.currentTextChanged.connect(self._set_colormap)

        for label, widget in (("Colour scale:", self.spin_fs), ("Colormap:", self.combo_cmap)):
            bar.addWidget(QtWidgets.QLabel(label))
            bar.addWidget(widget)
        bar.addStretch(1)
        self.lbl_state = QtWidgets.QLabel("idle")
        self.lbl_state.setStyleSheet(f"color: {theme.TEXT_DIM};")
        bar.addWidget(self.lbl_state)
        root.addLayout(bar)

        mid = QtWidgets.QSplitter(Qt.Orientation.Horizontal)

        self.plot = pg.PlotWidget()
        pi = self.plot.getPlotItem()
        pi.setTitle("Haptic output - 5 pneumatic valves", color=theme.TEXT, size="10pt")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.35)
        self.plot.setLabel("bottom", "x", units="mm")
        self.plot.setLabel("left", "y  (tip -> base)", units="mm")
        self.plot.setMenuEnabled(False)

        self._x = np.array([v[2] for v in VALVE_LAYOUT], dtype=float)
        self._y = np.array([v[3] for v in VALVE_LAYOUT], dtype=float)

        self.scatter = pg.ScatterPlotItem(
            size=VALVE_SIZE_MIN,
            pen=pg.mkPen(theme.BORDER_HOVER, width=1.0),
        )
        self.plot.addItem(self.scatter)
        self._texts = []
        for name, _label, x, y in VALVE_LAYOUT:
            txt = pg.TextItem(color=theme.TEXT, anchor=(0.5, 1.0))
            txt.setPos(x, y + 2.0)
            self.plot.addItem(txt)
            self._texts.append(txt)
        self.plot.setXRange(-POSITION_PAD - 4, POSITION_PAD + 4)
        self.plot.setYRange(-2.0, VALVE_LAYOUT[-1][3] + 5.0)
        mid.addWidget(self.plot)

        self.table = QtWidgets.QTableWidget(len(VALVE_LAYOUT), 6)
        self.table.setHorizontalHeaderLabels(
            ["Valve", "Position", "Force (N)", "Weight", "Byte", "Age (s)"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.NoSelection)
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeMode.Stretch)
        for r, (name, label, _x, _y) in enumerate(VALVE_LAYOUT):
            cells = [name, label, "---", "---", "---", "---"]
            for c, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                if c >= 2:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.table.setItem(r, c, item)
        mid.addWidget(self.table)
        mid.setStretchFactor(0, 3)
        mid.setStretchFactor(1, 2)
        root.addWidget(mid, 1)

        self.lbl_frame = QtWidgets.QLabel("frame: ---")
        self.lbl_frame.setStyleSheet(theme.VALUE_QSS)
        self.lbl_frame.setWordWrap(True)
        self.lbl_frame.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        root.addWidget(self.lbl_frame)

        self.update_snapshot(None)

    # -- setters ------------------------------------------------------------
    def _set_full_scale(self, value: float) -> None:
        self._full_scale = float(value)

    def _set_colormap(self, name: str) -> None:
        self.lut = _lookup_table(name)
        self._brush_key = None

    # -- update -------------------------------------------------------------
    def update_snapshot(self, snap: Optional[HapticSnapshot]) -> None:
        if snap is None or not snap.valves:
            self.scatter.setData(pos=np.column_stack([self._x, self._y]),
                                 brush=self.lut[0], size=VALVE_SIZE_MIN)
            self._brush_key = None
            for (name, _label, _x, _y), txt in zip(VALVE_LAYOUT, self._texts):
                txt.setText(name)
            self.lbl_state.setText("idle")
            self.lbl_frame.setText("frame: ---")
            self._fill_table(None)
            return

        scale = max(self._full_scale, 1e-3)
        forces = np.array([v.force for v in snap.valves], dtype=float)
        idx = np.clip(forces * (255.0 / scale), 0, 255).astype(np.int32)
        sizes = VALVE_SIZE_MIN + (VALVE_SIZE_MAX - VALVE_SIZE_MIN) * np.clip(forces / scale, 0, 1)

        key = idx.tobytes()
        if key != self._brush_key:
            self._brush_key = key
            self.scatter.setBrush([self.lut[i] for i in idx])
        self.scatter.setData(pos=np.column_stack([self._x, self._y]),
                             brush=[self.lut[i] for i in idx], size=sizes)

        for valve, txt in zip(snap.valves, self._texts):
            txt.setText(f"{valve.name}\n{valve.force:.1f} N\n{valve.age:.2f} s")

        state = snap.state
        if snap.error:
            state = f"{state}: {snap.error}"
        self.lbl_state.setText(state)
        self.lbl_frame.setText(
            f"frame: {snap.frame_hex}    seq {snap.seq}    "
            f"flags 0x{snap.flags:02X} [{', '.join(snap.flag_names) or 'none'}]    "
            f"{snap.rate_hz:.1f} Hz"
        )
        self._fill_table(snap)

    def _fill_table(self, snap: Optional[HapticSnapshot]) -> None:
        for r, (name, label, _x, _y) in enumerate(VALVE_LAYOUT):
            if snap is None or r >= len(snap.valves):
                values = [name, label, "---", "---", "---", "---"]
            else:
                v = snap.valves[r]
                values = [
                    v.name, v.label, fmt(v.force, "5.2f"), fmt(v.weight, "5.3f"),
                    str(v.raw), fmt(v.age, "5.2f"),
                ]
            for c, text in enumerate(values):
                self.table.item(r, c).setText(text)
