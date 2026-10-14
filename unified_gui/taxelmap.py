"""Live tactile taxel map (Phase 4.4, reference-style view).

This follows the taxel map in ``Sensor/paxini_usb_gui.py``: the 52 taxels are
drawn at their physical ``(x, y)`` coordinates on an aspect-locked pyqtgraph
plot, coloured by normal force through a lookup table, with the contact point
marked by a ``+`` and the tangential (shear) force drawn as a short arrow
starting at it.

pyqtgraph is GPU-drawn, so this view stays live at the full 30 FPS GUI refresh
rate - unlike the Matplotlib 3D canvas, which is CPU-rendered and throttled.
Both views read the same ``SensorReading`` snapshots.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np
from PyQt6 import QtWidgets
import pyqtgraph as pg

from . import theme
from .sensors import SensorLink

TAXEL_SIZE = 16.0
FULL_SCALE_MIN, FULL_SCALE_MAX = 0.1, 25.0
ARROW_SCALE_MIN, ARROW_SCALE_MAX = 0.5, 50.0
CMAPS = ("Greys", "cividis", "viridis", "inferno")


def _lookup_table(name: str):
    """256 brushes for a colormap name (matplotlib names included).

    pyqtgraph only bundles a handful of maps itself, so matplotlib is asked
    first - the names in ``CMAPS`` all come from there. A plain greyscale ramp
    is built by hand if neither source knows the name, so the map can never
    fail to draw.
    """
    lut = None
    for source in ("matplotlib", "sister"):
        try:
            lut = pg.colormap.get(name, source=source).getLookupTable(nPts=256, alpha=False)
            break
        except Exception:
            lut = None
    if lut is None:
        ramp = np.linspace(0.0, 1.0, 256)
        lut = np.column_stack([ramp, ramp, ramp])
    return [pg.mkBrush(int(r), int(g), int(b)) for r, g, b, *_ in lut]


class _SensorMap(QtWidgets.QWidget):
    """One finger: 52 taxels by position, coloured by normal force."""

    def __init__(self, tag: str, label: str, parent=None) -> None:
        super().__init__(parent)
        self.tag = tag
        self.lut = _lookup_table(theme.TAXEL_CMAP)
        self._brush_key: Optional[bytes] = None
        self._coords: Optional[np.ndarray] = None

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plot = pg.PlotWidget()
        pi = self.plot.getPlotItem()
        pi.setTitle(f"{label} finger - 52 taxels", color=theme.TEXT, size="9pt")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.35)
        self.plot.setLabel("bottom", "x", units="mm")
        self.plot.setLabel("left", "y", units="mm")
        self.plot.setMenuEnabled(False)

        self.scatter = pg.ScatterPlotItem(
            size=TAXEL_SIZE,
            pen=pg.mkPen(theme.BORDER_HOVER, width=0.7),
            hoverable=True,
            tip="Taxel #{data}\nx={x:.2f}  y={y:.2f} mm",
        )
        self.plot.addItem(self.scatter)
        self.vector = pg.PlotDataItem(pen=pg.mkPen(theme.TEXT, width=2.5))
        self.plot.addItem(self.vector)
        self.cop = pg.ScatterPlotItem(
            size=18, symbol="+", pen=pg.mkPen(theme.TEXT, width=2.5)
        )
        self.plot.addItem(self.cop)
        layout.addWidget(self.plot)

    # -- geometry -----------------------------------------------------------
    def set_coords(self, xyz: np.ndarray) -> None:
        self._coords = np.asarray(xyz, dtype=float)
        self.scatter.setData(
            pos=self._coords[:, :2],
            data=np.arange(1, len(self._coords) + 1),
            brush=self.lut[0],
        )
        self.plot.autoRange()

    def set_colormap(self, name: str) -> None:
        self.lut = _lookup_table(name)
        self._brush_key = None

    # -- per-frame update ---------------------------------------------------
    def update_map(self, reading, full_scale: float, arrow_mm_per_n: float) -> None:
        if self._coords is None or reading is None or reading.taxels is None:
            self.cop.setData([], [])
            self.vector.setData([], [])
            return

        fz = np.clip(np.asarray(reading.taxels, dtype=float)[:, 2], 0.0, None)
        scale = max(float(full_scale), 1e-3)
        # colour index per taxel; only repaint the brushes when they change
        idx = np.clip(fz * (255.0 / scale), 0, 255).astype(np.int32)
        key = idx.tobytes()
        if key != self._brush_key:
            self._brush_key = key
            self.scatter.setBrush([self.lut[i] for i in idx])

        cx, cy = reading.cop_x, reading.cop_y
        if cx == cx and cy == cy:
            self.cop.setData([cx], [cy])
            self.vector.setData(
                [cx, cx + reading.fx * arrow_mm_per_n],
                [cy, cy + reading.fy * arrow_mm_per_n],
            )
        else:
            self.cop.setData([], [])
            self.vector.setData([], [])


class TaxelMapView(QtWidgets.QWidget):
    """Both fingers side by side, with the shared colour/arrow scaling."""

    def __init__(self, links: Dict[str, SensorLink], parent=None) -> None:
        super().__init__(parent)
        self.links = dict(links)
        self._full_scale = 2.0
        self._arrow_scale = 5.0

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(3)

        bar = QtWidgets.QHBoxLayout()
        bar.setSpacing(6)
        self.spin_fs = QtWidgets.QDoubleSpinBox()
        self.spin_fs.setRange(FULL_SCALE_MIN, FULL_SCALE_MAX)
        self.spin_fs.setSingleStep(0.5)
        self.spin_fs.setValue(self._full_scale)
        self.spin_fs.setSuffix(" N")
        self.spin_fs.setFixedWidth(78)
        self.spin_fs.setToolTip("Taxel colour full-scale normal force Fz")
        self.spin_fs.valueChanged.connect(self.set_full_scale)

        self.spin_arrow = QtWidgets.QDoubleSpinBox()
        self.spin_arrow.setRange(ARROW_SCALE_MIN, ARROW_SCALE_MAX)
        self.spin_arrow.setSingleStep(1.0)
        self.spin_arrow.setValue(self._arrow_scale)
        self.spin_arrow.setSuffix(" mm/N")
        self.spin_arrow.setFixedWidth(88)
        self.spin_arrow.setToolTip("Shear-force arrow length per Newton")
        self.spin_arrow.valueChanged.connect(self.set_arrow_scale)

        self.combo_cmap = QtWidgets.QComboBox()
        self.combo_cmap.addItems(CMAPS)
        self.combo_cmap.setCurrentText(theme.TAXEL_CMAP if theme.TAXEL_CMAP in CMAPS else CMAPS[0])
        self.combo_cmap.setFixedWidth(96)
        self.combo_cmap.currentTextChanged.connect(self.set_colormap)

        for label, widget in (("Colour scale:", self.spin_fs),
                              ("Arrow:", self.spin_arrow),
                              ("Colormap:", self.combo_cmap)):
            bar.addWidget(QtWidgets.QLabel(label))
            bar.addWidget(widget)
        bar.addStretch(1)
        hint = QtWidgets.QLabel("+ CoP   ·   line = shear")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setToolTip("The + marks the centre of pressure on the sensor surface; "
                        "the line is the tangential (shear) force vector.")
        # let it clip rather than widen the whole window
        hint.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                           QtWidgets.QSizePolicy.Policy.Preferred)
        bar.addWidget(hint)
        root.addLayout(bar)

        grid = QtWidgets.QGridLayout()
        grid.setSpacing(4)
        self.maps: Dict[str, _SensorMap] = {}
        for col, (tag, label) in enumerate((("L", "Left"), ("R", "Right"))):
            pane = _SensorMap(tag, label)
            link = self.links.get(tag)
            pane.set_coords(link.xyz if link is not None else _default_xyz())
            self.maps[tag] = pane
            grid.addWidget(pane, 0, col)
            grid.setColumnStretch(col, 1)
        root.addLayout(grid, 1)

    # -- setters ------------------------------------------------------------
    def set_full_scale(self, value: float) -> None:
        self._full_scale = float(value)

    def set_arrow_scale(self, value: float) -> None:
        self._arrow_scale = float(value)

    def set_colormap(self, name: str) -> None:
        for pane in self.maps.values():
            pane.set_colormap(name)

    def set_links(self, links: Dict[str, SensorLink]) -> None:
        """Point the panes at (possibly newly connected) sensor links."""
        self.links = dict(links)
        for tag, pane in self.maps.items():
            link = self.links.get(tag)
            if link is not None:
                pane.set_coords(link.xyz)

    # -- update -------------------------------------------------------------
    def update_readings(self, readings: Dict[str, Optional[object]]) -> None:
        for tag, pane in self.maps.items():
            pane.update_map(readings.get(tag), self._full_scale, self._arrow_scale)


def _default_xyz() -> np.ndarray:
    from . import _paths  # noqa: F401
    import paxini_usb_core as core

    return np.asarray(core.S2015_ELITE_XYZ, dtype=float)
