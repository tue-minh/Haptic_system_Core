"""3D tactile point-cloud + force-vector view (Phase 4.4, 4.5).

A Matplotlib 3D canvas (embedded through ``FigureCanvasQTAgg``) renders one
sub-plot per finger:

* the 52 taxels of the S2015-Elite layout at their physical ``(x, y, z)``
  coordinates, coloured by the live normal force ``Fz_i`` (colormap) and sized
  proportionally to pressure;
* a 3D arrow rooted at the current Centre of Pressure pointing along
  ``[Fx, Fy, Fz] * scale`` whose length tracks the magnitude ``|F|``.

Redraw cost is kept low so the 3D view never starves the 30 FPS GUI loop:

* point coordinates never change, so only the colour array and point sizes are
  updated on the existing scatter artist;
* the arrow is a reused ``Line3D`` (plus a tip marker) rather than a freshly
  built ``quiver`` per frame;
* the view throttles itself to ``update_hz`` (default 6 Hz, user adjustable)
  and leaves the colour limits untouched while the full-scale is unchanged.
"""
from __future__ import annotations

import time
from typing import Dict, Optional

import numpy as np

from PyQt6 import QtWidgets

from . import theme
from .sensors import SensorLink

try:  # Matplotlib is a hard requirement; deps.verify() reports it if missing.
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

    _MPL_OK = True
except Exception:  # pragma: no cover - depends on env
    _MPL_OK = False
    FigureCanvasQTAgg = object  # type: ignore

BASE_TAXEL_SIZE = 24.0
SIZE_GAIN = 320.0
FULL_SCALE_MIN, FULL_SCALE_MAX = 0.1, 25.0
ARROW_SCALE_MIN, ARROW_SCALE_MAX = 0.5, 50.0
UPDATE_HZ_MIN, UPDATE_HZ_MAX = 1.0, 30.0
DEFAULT_UPDATE_HZ = 4.0
# The colormap is the one place the 3D view can use colour: it encodes
# pressure. Reversed greyscale is the default (dark = no load, bright = loaded);
# hue maps remain selectable.
CMAPS = ("Greys_r", "cividis", "viridis", "inferno")


class _Finger3D:
    """One 3D axis with its reused taxel scatter, CoP marker and arrow line."""

    def __init__(self, ax, xyz: np.ndarray, title: str, cmap: str,
                 full_scale: float, color: str):
        self.ax = ax
        self.xyz = np.asarray(xyz, dtype=float)
        self.title = title
        self.color = color
        self._clim = (0.0, float(full_scale))
        self._has_cop = False

        self.ax.set_title(title, color=theme.TEXT, fontsize=9, fontweight="bold", pad=2)
        self.ax.set_xlabel("X (mm)", color=theme.TEXT_DIM, fontsize=7, labelpad=-4)
        self.ax.set_ylabel("Y (mm)", color=theme.TEXT_DIM, fontsize=7, labelpad=-4)
        self.ax.set_zlabel("Z (mm)", color=theme.TEXT_DIM, fontsize=7, labelpad=-6)

        self.scatter = self.ax.scatter(
            self.xyz[:, 0], self.xyz[:, 1], self.xyz[:, 2],
            s=np.full(len(self.xyz), BASE_TAXEL_SIZE),
            c=np.zeros(len(self.xyz)),
            cmap=cmap, vmin=0.0, vmax=float(full_scale),
            depthshade=False, edgecolors=theme.BORDER_HOVER, linewidths=0.6,
        )
        self.cop = self.ax.scatter(
            [], [], [], marker="+", s=140, c=color, depthshade=False, linewidths=2.0,
        )
        self.arrow, = self.ax.plot3D(
            [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], color=color, lw=2.4,
        )
        self.arrow_tip, = self.ax.plot3D(
            [0.0], [0.0], [0.0], linestyle="none", marker="o", markersize=4.0, color=color,
        )
        self._set_arrow_visible(False)
        self._style()
        self._fix_limits()

    # -- static styling -----------------------------------------------------
    def _style(self) -> None:
        self.ax.set_facecolor(theme.PANEL)
        for axis in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            axis.pane.set_facecolor((0.945, 0.952, 0.963, 1.0))
            axis.pane.set_edgecolor(theme.BORDER)
            # Tick-label auto-rotation is the single most expensive part of a
            # Matplotlib 3D draw; disabling it roughly halves the frame cost.
            try:
                axis.set_rotate_label(False)
            except Exception:  # pragma: no cover - mpl < 3.9
                pass
        self.ax.tick_params(colors=theme.TEXT_DIM, labelsize=6, pad=0)
        self.ax.grid(True, color=theme.BORDER, alpha=0.7, linewidth=0.4)
        # a handful of ticks instead of matplotlib's crowded automatic ticks
        self.ax.set_xticks([-8, 0, 8])
        self.ax.set_yticks([0, 10, 20])
        self.ax.set_zticks([2, 4, 6])

    def _fix_limits(self) -> None:
        pad = 2.5
        self.ax.set_xlim3d(self.xyz[:, 0].min() - pad, self.xyz[:, 0].max() + pad)
        self.ax.set_ylim3d(self.xyz[:, 1].min() - pad, self.xyz[:, 1].max() + pad)
        self.ax.set_zlim3d(self.xyz[:, 2].min() - pad, self.xyz[:, 2].max() + pad)
        self.ax.set_autoscale_on(False)
        try:
            self.ax.set_box_aspect((1.0, 2.1, 0.7))
        except Exception:  # pragma: no cover - very old matplotlib
            pass

    def set_cmap(self, cmap: str) -> None:
        self.scatter.set_cmap(cmap)

    # -- per-frame update ---------------------------------------------------
    def update(self, reading, full_scale: float, arrow_scale: float) -> None:
        n = len(self.xyz)
        if reading is None or reading.taxels is None:
            self.scatter.set_sizes(np.full(n, BASE_TAXEL_SIZE))
            self._clear_dynamic()
            return

        tax = np.asarray(reading.taxels, dtype=np.float64)
        fz = np.clip(tax[:, 2], 0.0, None) if tax.shape == (n, 3) else np.zeros(n)

        scale = max(float(full_scale), 1e-3)
        self.scatter.set_array(fz)
        if self._clim[1] != scale:
            self._clim = (0.0, scale)
            self.scatter.set_clim(0.0, scale)
        self.scatter.set_sizes(BASE_TAXEL_SIZE + SIZE_GAIN * np.clip(fz / scale, 0.0, 1.0))

        cx, cy = reading.cop_x, reading.cop_y
        if not (cx == cx and cy == cy):
            self._clear_dynamic()
            return

        w_sum = float(fz.sum())
        cz = float((fz @ self.xyz[:, 2]) / w_sum) if w_sum > 1e-9 else float(self.xyz[:, 2].mean())
        self.cop._offsets3d = (np.array([cx]), np.array([cy]), np.array([cz]))
        self._has_cop = True

        vec = np.array([reading.fx, reading.fy, reading.fz], dtype=float)
        if not np.all(np.isfinite(vec)) or float(np.linalg.norm(vec)) < 1e-6:
            self._set_arrow_visible(False)
            return
        dx, dy, dz = vec * arrow_scale
        self.arrow.set_data_3d([cx, cx + dx], [cy, cy + dy], [cz, cz + dz])
        self.arrow_tip.set_data_3d([cx + dx], [cy + dy], [cz + dz])
        self._set_arrow_visible(True)

    def _clear_dynamic(self) -> None:
        if self._has_cop:
            self.cop._offsets3d = (np.empty(0), np.empty(0), np.empty(0))
            self._has_cop = False
        self._set_arrow_visible(False)

    def _set_arrow_visible(self, on: bool) -> None:
        self.arrow.set_visible(on)
        self.arrow_tip.set_visible(on)


class Tactile3DView(QtWidgets.QWidget):
    """Embedded 3D view showing both fingers side by side."""

    def __init__(self, links: Dict[str, SensorLink], parent=None) -> None:
        super().__init__(parent)
        self.links = dict(links)
        self._cmap = theme.TAXEL_CMAP
        self._full_scale = 2.0
        self._arrow_scale = 5.0
        self._update_hz = DEFAULT_UPDATE_HZ
        self._last_update = 0.0
        self._live = True
        self._drew_data = False

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(3)

        bar = QtWidgets.QVBoxLayout()
        bar.setSpacing(4)
        row_scale = QtWidgets.QHBoxLayout()
        row_view = QtWidgets.QHBoxLayout()

        self.spin_fs = QtWidgets.QDoubleSpinBox()
        self.spin_fs.setRange(FULL_SCALE_MIN, FULL_SCALE_MAX)
        self.spin_fs.setSingleStep(0.5)
        self.spin_fs.setValue(self._full_scale)
        self.spin_fs.setSuffix(" N")
        self.spin_fs.setFixedWidth(78)
        self.spin_fs.setToolTip("Taxel colour / size full-scale normal force Fz")
        self.spin_fs.valueChanged.connect(self.set_full_scale)

        self.spin_arrow = QtWidgets.QDoubleSpinBox()
        self.spin_arrow.setRange(ARROW_SCALE_MIN, ARROW_SCALE_MAX)
        self.spin_arrow.setSingleStep(1.0)
        self.spin_arrow.setValue(self._arrow_scale)
        self.spin_arrow.setSuffix(" mm/N")
        self.spin_arrow.setFixedWidth(88)
        self.spin_arrow.setToolTip("Force-vector arrow length per Newton")
        self.spin_arrow.valueChanged.connect(self.set_arrow_scale)

        self.spin_hz = QtWidgets.QDoubleSpinBox()
        self.spin_hz.setRange(UPDATE_HZ_MIN, UPDATE_HZ_MAX)
        self.spin_hz.setSingleStep(1.0)
        self.spin_hz.setValue(self._update_hz)
        self.spin_hz.setSuffix(" Hz")
        self.spin_hz.setFixedWidth(72)
        self.spin_hz.setToolTip(
            "3D redraw rate. Matplotlib 3D is CPU-rendered - lower this if the "
            "GUI feels sluggish (installing PyOpenGL enables GPU rendering).")
        self.spin_hz.valueChanged.connect(self._set_rate)

        self.chk_live = QtWidgets.QCheckBox("Live")
        self.chk_live.setChecked(True)
        self.chk_live.setToolTip("Pause/resume 3D redraws (freezes the current frame)")
        self.chk_live.toggled.connect(self._set_live)

        self.combo_cmap = QtWidgets.QComboBox()
        self.combo_cmap.addItems(CMAPS)
        self.combo_cmap.setCurrentText(self._cmap)
        self.combo_cmap.setFixedWidth(96)
        self.combo_cmap.currentTextChanged.connect(self.set_cmap)

        for label, widget in (("Colour scale:", self.spin_fs), ("Arrow:", self.spin_arrow)):
            row_scale.addWidget(QtWidgets.QLabel(label))
            row_scale.addWidget(widget)
        row_scale.addStretch(1)

        for label, widget in (("Update:", self.spin_hz),
                              ("Colormap:", self.combo_cmap),
                              ("", self.chk_live)):
            if label:
                row_view.addWidget(QtWidgets.QLabel(label))
            row_view.addWidget(widget)
        row_view.addStretch(1)

        bar.addLayout(row_scale)
        bar.addLayout(row_view)
        root.addLayout(bar)

        if not _MPL_OK:
            warn = QtWidgets.QLabel(
                "Matplotlib 3D view unavailable - install matplotlib to enable it.")
            warn.setStyleSheet(f"color: {theme.TEXT_DIM};")
            root.addWidget(warn)
            self.canvas = None
            self.figure = None
            self._panes: Dict[str, _Finger3D] = {}
            return

        self.figure = Figure(figsize=(9, 4))
        self.figure.patch.set_facecolor(theme.BG)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setStyleSheet(f"background-color: {theme.BG};")
        ax_l = self.figure.add_subplot(1, 2, 1, projection="3d")
        ax_r = self.figure.add_subplot(1, 2, 2, projection="3d")
        self.figure.subplots_adjust(left=0.02, right=0.86, bottom=0.02, top=0.94, wspace=0.05)

        self._panes = {}
        for tag, ax, label in (("L", ax_l, "Left"), ("R", ax_r, "Right")):
            link = self.links.get(tag)
            xyz = link.xyz if link is not None else _default_xyz()
            self._panes[tag] = _Finger3D(ax, xyz, f"{label} finger - 52 taxels",
                                         self._cmap, self._full_scale, theme.TEXT)

        cax = self.figure.add_axes([0.885, 0.15, 0.022, 0.7])
        self._colorbar = self.figure.colorbar(self._panes["L"].scatter, cax=cax)
        self._colorbar.set_label("Taxel Fz (N)", color=theme.TEXT_DIM, fontsize=8)
        self._colorbar.ax.tick_params(colors=theme.TEXT_DIM, labelsize=6)
        self._colorbar.outline.set_edgecolor(theme.BORDER)
        root.addWidget(self.canvas, 1)

    # -- setters ------------------------------------------------------------
    def set_full_scale(self, value: float) -> None:
        self._full_scale = float(value)

    def set_arrow_scale(self, value: float) -> None:
        self._arrow_scale = float(value)

    def set_cmap(self, name: str) -> None:
        self._cmap = name
        for pane in self._panes.values():
            pane.set_cmap(name)

    def set_links(self, links: Dict[str, SensorLink]) -> None:
        """Point the panes at (possibly newly connected) sensor links."""
        self.links = dict(links)
        for tag, pane in self._panes.items():
            link = self.links.get(tag)
            if link is not None:
                pane.xyz = link.xyz

    def _set_rate(self, hz: float) -> None:
        self._update_hz = max(UPDATE_HZ_MIN, float(hz))

    def _set_live(self, on: bool) -> None:
        self._live = bool(on)

    # -- update -------------------------------------------------------------
    def update_readings(self, readings: Dict[str, Optional[object]]) -> None:
        """Redraw at most ``update_hz`` times per second (self-throttling).

        Skips the (expensive) redraw entirely while no sensor is delivering
        data, so an idle or half-empty GUI stays perfectly responsive.
        """
        if self.canvas is None or not self._live:
            return
        has_data = any(r is not None for r in readings.values())
        if not has_data and not self._drew_data:
            return
        now = time.perf_counter()
        if now - self._last_update < 1.0 / self._update_hz:
            return
        self._last_update = now
        self._drew_data = has_data
        for tag, pane in self._panes.items():
            pane.update(readings.get(tag), self._full_scale, self._arrow_scale)
        self.canvas.draw_idle()


def _default_xyz() -> np.ndarray:
    from . import _paths  # noqa: F401
    import paxini_usb_core as core

    return np.asarray(core.S2015_ELITE_XYZ, dtype=float)
