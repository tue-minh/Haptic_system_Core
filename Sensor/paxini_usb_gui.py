"""
paxini_multi_gui.py
===================
PyQt6 GUI for reading several PaXini GEN3 tactile sensors (e.g. S2015-Elite)
at the same time, each on its own High-Speed Communication Board / COM port.

Features
  * any number of COM ports (one panel per port, "+ Add sensor")
  * live readouts: contact location (centre of pressure), contact-point force
    (peak taxel), force direction / orientation, force magnitude, ...
  * Start/Stop recording -> one CSV per sensor in this folder
  * check boxes select which channels appear in the real-time plot
  * taxel pressure map with CoP marker and shear-force arrow

Run:   python paxini_multi_gui.py
Needs: pip install PyQt6 pyqtgraph pyserial numpy
"""
from __future__ import annotations

import datetime
import os
import queue
import sys

import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt
import pyqtgraph as pg

import paxini_usb_core as core

RECORD_DIR = os.path.dirname(os.path.abspath(__file__))

# key, label, ring column, plot group, colour, checked by default
CHANNELS = [
    ("Fx",        "Fx (N)",              core.COL["Fx"],        "force",    "#ff5a5a", True),
    ("Fy",        "Fy (N)",              core.COL["Fy"],        "force",    "#5aff7a", True),
    ("Fz",        "Fz (N)",              core.COL["Fz"],        "force",    "#5aa0ff", True),
    ("F_mag",     "|F| magnitude (N)",   core.COL["F_mag"],     "force",    "#ffb347", False),
    ("F_shear",   "Shear Ft (N)",        core.COL["F_shear"],   "force",    "#d58cff", False),
    ("peak_Fz",   "Contact-pt Fz (N)",   core.COL["peak_Fz"],   "force",    "#40e0e0", False),
    ("CoP_x",     "Contact x (mm)",      core.COL["CoP_x"],     "position", "#ff70d0", False),
    ("CoP_y",     "Contact y (mm)",      core.COL["CoP_y"],     "position", "#d0ff40", False),
    ("shear_dir", "Shear direction (°)", core.COL["shear_dir"], "angle",    "#30c0a0", False),
    ("tilt",      "Tilt from normal (°)", core.COL["tilt"],     "angle",    "#e0a060", False),
    ("mu",        "Ft/Fz ratio",         core.COL["mu"],        "misc",     "#ff8080", False),
    ("contact_n", "Contact taxels",      core.COL["contact_n"], "misc",     "#c0c060", False),
    ("peak_idx",  "Contact taxel #",     core.COL["peak_idx"],  "misc",     "#a0a0ff", False),
]
CH = {c[0]: c for c in CHANNELS}
PLOT_GROUPS = [("force", "Force [N]"), ("position", "Contact location [mm]"),
               ("angle", "Direction [deg]"), ("misc", "Other")]
LINE_STYLES = [Qt.PenStyle.SolidLine, Qt.PenStyle.DashLine,
               Qt.PenStyle.DotLine, Qt.PenStyle.DashDotLine]
SENSOR_TAGS = ["#ff6060", "#60c0ff", "#80ff80", "#ffd060", "#d080ff", "#60ffe0"]


def fmt(v, spec="+7.2f", unit=""):
    if v is None or v != v:     # None or NaN
        return "   ---"
    return f"{v:{spec}}{unit}"


# ==========================================================================
# Taxel map widget
# ==========================================================================
class TaxelMap(pg.PlotWidget):
    def __init__(self, title: str):
        super().__init__()
        self.setTitle(title)
        self.setAspectLocked(True)
        self.showGrid(x=True, y=True, alpha=0.2)
        self.setLabel("bottom", "x", units="mm")
        self.setLabel("left", "y", units="mm")
        self.setMenuEnabled(False)
        lut = pg.colormap.get("inferno").getLookupTable(nPts=256, alpha=False)
        self.lut = [pg.mkBrush(int(r), int(g), int(b)) for r, g, b in lut]
        self.scatter = pg.ScatterPlotItem(size=17, pen=pg.mkPen("#888", width=0.6),
                                          hoverable=True,
                                          tip="Taxel #{data}\nx={x:.2f} y={y:.2f}".format)
        self.addItem(self.scatter)
        self.vec = pg.PlotDataItem(pen=pg.mkPen("#40ff40", width=3))
        self.addItem(self.vec)
        self.cop = pg.ScatterPlotItem(size=18, symbol="+", pen=pg.mkPen("#00ffff", width=3))
        self.addItem(self.cop)
        self.xy = None
        self._last_idx = None

    def set_coords(self, xy: np.ndarray):
        self.xy = xy
        self._last_idx = None
        self.scatter.setData(pos=xy, data=np.arange(1, len(xy) + 1), brush=self.lut[0])
        self.autoRange()

    def update_map(self, tax, row, full_scale: float, arrow_mm_per_n: float):
        if tax is None or self.xy is None or len(tax) != len(self.xy):
            return
        idx = np.clip(tax[:, 2] * (255.0 / full_scale), 0, 255).astype(np.int32)
        if self._last_idx is None or not np.array_equal(idx, self._last_idx):
            self._last_idx = idx
            self.scatter.setBrush([self.lut[i] for i in idx])
        cx, cy = row[core.COL["CoP_x"]], row[core.COL["CoP_y"]]
        if cx == cx:
            self.cop.setData([cx], [cy])
            fx, fy = row[core.COL["Fx"]], row[core.COL["Fy"]]
            self.vec.setData([cx, cx + fx * arrow_mm_per_n], [cy, cy + fy * arrow_mm_per_n])
        else:
            self.cop.clear()
            self.vec.setData([], [])


# ==========================================================================
# One panel per COM port
# ==========================================================================
class SensorPanel(QtWidgets.QGroupBox):
    def __init__(self, idx: int, main: "MainWindow"):
        super().__init__()
        self.idx = idx
        self.main = main
        self.worker: core.UsbSensorWorker | None = None
        self.curves: dict[str, pg.PlotDataItem] = {}
        self._modules_shown = 1
        self.tag = f"S{idx + 1}"
        self.color = SENSOR_TAGS[idx % len(SENSOR_TAGS)]
        self.setTitle(f"Sensor {self.tag}")
        self.setStyleSheet(f"QGroupBox {{ border: 2px solid {self.color}; border-radius: 6px;"
                           f" margin-top: 10px; font-weight: bold; }}"
                           f"QGroupBox::title {{ subcontrol-origin: margin; left: 8px;"
                           f" color: {self.color}; }}")
        self.map = TaxelMap(f"Sensor {self.tag}")

        lay = QtWidgets.QVBoxLayout(self)

        # --- connection row
        row = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        self.btn_remove = QtWidgets.QPushButton("x")
        self.btn_remove.setFixedWidth(28)
        self.btn_remove.setToolTip("Remove this sensor panel")
        self.btn_remove.clicked.connect(lambda: self.main.remove_panel(self))
        row.addWidget(QtWidgets.QLabel("Port:"))
        row.addWidget(self.port_combo, 1)
        row.addWidget(self.btn_connect)
        row.addWidget(self.btn_remove)
        lay.addLayout(row)

        row2 = QtWidgets.QHBoxLayout()
        self.module_combo = QtWidgets.QComboBox()
        self.module_combo.currentIndexChanged.connect(self.on_module_changed)
        self.btn_tare = QtWidgets.QPushButton("Tare")
        self.btn_tare.setToolTip("Software zero: average the next 50 frames and subtract")
        self.btn_tare.clicked.connect(self.tare)
        self.btn_untare = QtWidgets.QPushButton("Reset tare")
        self.btn_untare.clicked.connect(self.untare)
        row2.addWidget(QtWidgets.QLabel("Module:"))
        row2.addWidget(self.module_combo, 1)
        row2.addWidget(self.btn_tare)
        row2.addWidget(self.btn_untare)
        lay.addLayout(row2)

        self.status = QtWidgets.QLabel("Disconnected")
        self.status.setStyleSheet("color: gray; font-weight: normal;")
        lay.addWidget(self.status)

        # --- readouts
        mono = QtGui.QFont("Consolas", 9)
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        self.vals: dict[str, QtWidgets.QLabel] = {}
        items = [
            ("Force magnitude", [("|F|", "F_mag"), ("Fz (normal)", "Fz"), ("Ft (shear)", "F_shear")]),
            ("Force vector", [("Fx", "Fx"), ("Fy", "Fy")]),
            ("Direction / orientation", [("Shear dir.", "shear_dir"), ("Tilt", "tilt")]),
            ("Contact location", [("CoP x", "CoP_x"), ("CoP y", "CoP_y"), ("Taxel #", "peak_idx"),
                                  ("Contact taxels", "contact_n")]),
            ("Contact-point force", [("Peak Fz", "peak_Fz"), ("Ft/Fz", "mu")]),
        ]
        r = 0
        for title, fields in items:
            h = QtWidgets.QLabel(title)
            h.setStyleSheet("color:#9ab; font-weight: bold;")
            grid.addWidget(h, r, 0, 1, 4)
            r += 1
            for k, (lab, key) in enumerate(fields):
                l1 = QtWidgets.QLabel(lab)
                l1.setStyleSheet("font-weight: normal;")
                v = QtWidgets.QLabel("   ---")
                v.setFont(mono)
                v.setMinimumWidth(85)
                v.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                grid.addWidget(l1, r + k // 2, (k % 2) * 2)
                grid.addWidget(v, r + k // 2, (k % 2) * 2 + 1)
                self.vals[key] = v
            r += (len(fields) + 1) // 2
        lay.addLayout(grid)

        # --- plot channel check boxes
        box = QtWidgets.QGroupBox("Show in real-time plot")
        box.setStyleSheet("QGroupBox { border: 1px solid #555; font-weight: normal; margin-top: 8px; }"
                          "QGroupBox::title { subcontrol-origin: margin; left: 6px; }")
        cg = QtWidgets.QGridLayout(box)
        self.checks: dict[str, QtWidgets.QCheckBox] = {}
        for k, (key, label, _col, _grp, color, default) in enumerate(CHANNELS):
            cb = QtWidgets.QCheckBox(label)
            cb.setStyleSheet(f"QCheckBox {{ color: {color}; font-weight: normal; }}")
            cb.setChecked(default)
            cb.toggled.connect(lambda on, key=key: self.main.set_channel(self, key, on))
            cg.addWidget(cb, k // 2, k % 2)
            self.checks[key] = cb
        lay.addWidget(box)

    # ------------------------------------------------------------------
    def set_ports(self, ports, preferred: str | None = None):
        cur = self.port_combo.currentData() or preferred
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        for p in ports:
            self.port_combo.addItem(f"{p.device} - {p.description}", p.device)
        if cur:
            i = self.port_combo.findData(cur)
            if i >= 0:
                self.port_combo.setCurrentIndex(i)
        self.port_combo.blockSignals(False)

    def port(self) -> str:
        return self.port_combo.currentData() or self.port_combo.currentText().split(" ")[0]

    def is_active(self) -> bool:
        return self.worker is not None and self.worker.is_alive()

    def module(self):
        w = self.worker
        if w is None or w.state != "streaming":
            return None
        return w

    def toggle_connect(self):
        if self.is_active():
            self.disconnect_port()
            return
        port = self.port()
        if not port:
            self.main.post_log("No COM port selected")
            return
        if port in self.main.ports_in_use(exclude=self):
            QtWidgets.QMessageBox.warning(self, "Port busy", f"{port} is already used by another panel.")
            return
        self.worker = core.UsbSensorWorker(port, self.main.post_log, device_addr=1, model="S2015_elite")
        self.worker.start()
        self._modules_shown = 0
        self.module_combo.clear()
        self.btn_connect.setText("Disconnect")
        self.port_combo.setEnabled(False)
        self.setTitle(f"Sensor {self.tag} - {port}")
        self.map.setTitle(f"Sensor {self.tag} - {port}")

    def disconnect_port(self, wait: bool = False):
        if self.worker is not None:
            self.worker.stop()
            if wait:
                self.worker.join(timeout=1.5)

    def on_module_changed(self, _i):
        m = self.module()
        if m is not None:
            self.map.set_coords(m.xy)

    def tare(self):
        m = self.module()
        if m:
            m.request_tare()
            self.main.post_log(f"[{m.port}] Tare requested")

    def untare(self):
        m = self.module()
        if m:
            m.reset_tare()

    # ------------------------------------------------------------------
    def refresh_status(self):
        w = self.worker
        if w is None:
            return
        if w.state == "streaming":
            self.status.setText(f"Streaming {w.rate_hz:6.1f} Hz | frames {w.frames}")
            self.status.setStyleSheet("color: #3c3; font-weight: normal;")
        elif w.state == "connecting":
            self.status.setText("Connecting / reading configuration...")
            self.status.setStyleSheet("color: orange; font-weight: normal;")
        else:
            if not w.is_alive():
                txt = f"Error: {w.error}" if w.state == "error" else "Disconnected"
                self.status.setText(txt)
                self.status.setStyleSheet(f"color: {'#e44' if w.state == 'error' else 'gray'};"
                                          " font-weight: normal;")
                self.worker = None
                self.btn_connect.setText("Connect")
                self.port_combo.setEnabled(True)
                self.module_combo.clear()
                self._modules_shown = 0
                for c in self.curves.values():
                    c.setData([], [])

    def refresh_values(self):
        m = self.module()
        row = m.latest_row if m else None
        if row is None:
            return
        C = core.COL
        v = self.vals
        v["F_mag"].setText(fmt(row[C["F_mag"]], "7.2f", " N"))
        v["Fz"].setText(fmt(row[C["Fz"]], "+7.2f", " N"))
        v["F_shear"].setText(fmt(row[C["F_shear"]], "7.2f", " N"))
        v["Fx"].setText(fmt(row[C["Fx"]], "+7.2f", " N"))
        v["Fy"].setText(fmt(row[C["Fy"]], "+7.2f", " N"))
        v["shear_dir"].setText(fmt(row[C["shear_dir"]], "+7.1f", " °"))
        v["tilt"].setText(fmt(row[C["tilt"]], "7.1f", " °"))
        v["CoP_x"].setText(fmt(row[C["CoP_x"]], "+7.2f", " mm"))
        v["CoP_y"].setText(fmt(row[C["CoP_y"]], "+7.2f", " mm"))
        v["peak_idx"].setText(fmt(row[C["peak_idx"]], "5.0f"))
        v["contact_n"].setText(fmt(row[C["contact_n"]], "5.0f"))
        v["peak_Fz"].setText(fmt(row[C["peak_Fz"]], "7.2f", " N"))
        v["mu"].setText(fmt(row[C["mu"]], "7.3f"))


# ==========================================================================
# Main window
# ==========================================================================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PaXini GEN3 Multi-Sensor Monitor (S2015-Elite)")
        self.resize(1600, 950)
        self.panels: list[SensorPanel] = []
        self._log_q: "queue.SimpleQueue[str]" = queue.SimpleQueue()
        self.recorders: list[tuple[core.ModuleState, core.CsvRecorder]] = []
        self.rec_t0 = 0.0
        self._tick = 0

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vl = QtWidgets.QVBoxLayout(central)

        # ---------------- toolbar
        tb = QtWidgets.QHBoxLayout()
        b_add = QtWidgets.QPushButton("+ Add sensor")
        b_add.clicked.connect(lambda: self.add_panel())
        b_ref = QtWidgets.QPushButton("↻ Refresh ports")
        b_ref.clicked.connect(self.refresh_ports)
        b_ca = QtWidgets.QPushButton("Connect all")
        b_ca.clicked.connect(self.connect_all)
        b_da = QtWidgets.QPushButton("Disconnect all")
        b_da.clicked.connect(self.disconnect_all)
        self.btn_rec = QtWidgets.QPushButton("●  Start recording")
        self.btn_rec.setCheckable(True)
        self.btn_rec.setMinimumWidth(190)
        self.btn_rec.toggled.connect(self.toggle_record)
        self._rec_style_idle = "QPushButton { font-weight: bold; padding: 4px 10px; }"
        self.btn_rec.setStyleSheet(self._rec_style_idle)
        self.rec_label = QtWidgets.QLabel("Not recording")

        self.spin_window = QtWidgets.QDoubleSpinBox()
        self.spin_window.setRange(1, 60)
        self.spin_window.setValue(10)
        self.spin_window.setSuffix(" s")
        self.spin_fps = QtWidgets.QSpinBox()
        self.spin_fps.setRange(5, 60)
        self.spin_fps.setValue(30)
        self.spin_fps.setSuffix(" fps")
        self.spin_fps.valueChanged.connect(lambda v: self.timer.setInterval(int(1000 / v)))
        self.spin_thr = QtWidgets.QDoubleSpinBox()
        self.spin_thr.setRange(0.0, 5.0)
        self.spin_thr.setSingleStep(0.1)
        self.spin_thr.setValue(core.CONFIG.taxel_threshold_n)
        self.spin_thr.setSuffix(" N")
        self.spin_thr.setToolTip("Taxel contact threshold (used for contact location / taxel count)")
        self.spin_thr.valueChanged.connect(lambda v: setattr(core.CONFIG, "taxel_threshold_n", v))
        self.chk_pause = QtWidgets.QCheckBox("Pause plot")

        for w in (b_add, b_ref, b_ca, b_da):
            tb.addWidget(w)
        tb.addSpacing(20)
        tb.addWidget(self.btn_rec)
        tb.addWidget(self.rec_label)
        tb.addStretch(1)
        for lab, w in (("Window", self.spin_window), ("Refresh", self.spin_fps),
                       ("Contact thr.", self.spin_thr)):
            tb.addWidget(QtWidgets.QLabel(lab))
            tb.addWidget(w)
        tb.addWidget(self.chk_pause)
        vl.addLayout(tb)

        # ---------------- body
        vsplit = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        hsplit = QtWidgets.QSplitter(Qt.Orientation.Horizontal)

        self.panel_host = QtWidgets.QWidget()
        self.panel_layout = QtWidgets.QVBoxLayout(self.panel_host)
        self.panel_layout.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.panel_host)
        scroll.setMinimumWidth(430)
        hsplit.addWidget(scroll)

        self.tabs = QtWidgets.QTabWidget()
        # time-series tab
        ts = QtWidgets.QSplitter(Qt.Orientation.Vertical)
        self.plots: dict[str, pg.PlotWidget] = {}
        first = None
        for grp, title in PLOT_GROUPS:
            pw = pg.PlotWidget()
            pi = pw.getPlotItem()
            pi.setTitle(title)
            pi.showGrid(x=True, y=True, alpha=0.25)
            pi.addLegend(offset=(5, 5), labelTextSize="8pt")
            pi.setDownsampling(auto=True, mode="peak")
            pi.setClipToView(True)
            pi.enableAutoRange(axis="x", enable=False)
            pi.setLabel("bottom", "time", units="s")
            if first is None:
                first = pw
            else:
                pw.setXLink(first)
            self.plots[grp] = pw
            ts.addWidget(pw)
        self.tabs.addTab(ts, "Real-time plot")
        # taxel map tab
        mw = QtWidgets.QWidget()
        ml = QtWidgets.QVBoxLayout(mw)
        mtb = QtWidgets.QHBoxLayout()
        self.spin_fs = QtWidgets.QDoubleSpinBox()
        self.spin_fs.setRange(0.1, 25.5)
        self.spin_fs.setValue(2.0)
        self.spin_fs.setSuffix(" N")
        self.spin_arrow = QtWidgets.QDoubleSpinBox()
        self.spin_arrow.setRange(0.1, 50)
        self.spin_arrow.setValue(3.0)
        self.spin_arrow.setSuffix(" mm/N")
        mtb.addWidget(QtWidgets.QLabel("Taxel colour full scale (Fz):"))
        mtb.addWidget(self.spin_fs)
        mtb.addWidget(QtWidgets.QLabel("   Shear arrow scale:"))
        mtb.addWidget(self.spin_arrow)
        mtb.addWidget(QtWidgets.QLabel("   cyan + = contact location (CoP), green line = shear force"))
        mtb.addStretch(1)
        ml.addLayout(mtb)
        self.map_grid = QtWidgets.QGridLayout()
        ml.addLayout(self.map_grid, 1)
        self.tabs.addTab(mw, "Taxel map")
        hsplit.addWidget(self.tabs)
        hsplit.setStretchFactor(1, 1)
        vsplit.addWidget(hsplit)

        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setFont(QtGui.QFont("Consolas", 9))
        vsplit.addWidget(self.log_view)
        vsplit.setStretchFactor(0, 1)
        vsplit.setSizes([780, 140])
        vl.addWidget(vsplit, 1)

        # ---------------- start
        self.add_panel()
        self.add_panel()
        self.refresh_ports(auto_assign=True)
        for p in self.panels:
            for key, cb in p.checks.items():
                if cb.isChecked():
                    self.set_channel(p, key, True)
        self._update_plot_visibility()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(1000 / self.spin_fps.value()))
        self.timer.timeout.connect(self.on_tick)
        self.timer.start()
        self.post_log(f"Ready. CSV files will be saved to: {RECORD_DIR}")

    # ------------------------------------------------------------------
    # logging (thread-safe: workers only push into a queue)
    def post_log(self, msg: str):
        self._log_q.put(f"{datetime.datetime.now():%H:%M:%S}  {msg}")

    def _drain_log(self):
        lines = []
        try:
            while True:
                lines.append(self._log_q.get_nowait())
        except queue.Empty:
            pass
        if lines:
            self.log_view.appendPlainText("\n".join(lines))

    # ------------------------------------------------------------------
    # panels
    def add_panel(self):
        idx = 0
        used = {p.idx for p in self.panels}
        while idx in used:
            idx += 1
        p = SensorPanel(idx, self)
        self.panels.append(p)
        self.panels.sort(key=lambda q: q.idx)
        self.panel_layout.insertWidget(self.panels.index(p), p)
        p.set_ports(core.list_ports())
        if hasattr(self, "timer"):
            for key, cb in p.checks.items():
                if cb.isChecked():
                    self.set_channel(p, key, True)
            self._update_plot_visibility()
        self._relayout_maps()
        return p

    def remove_panel(self, p: SensorPanel):
        if self.btn_rec.isChecked() and p.is_active():
            QtWidgets.QMessageBox.information(self, "Recording", "Stop recording first.")
            return
        p.disconnect_port(wait=True)
        for key in list(p.curves):
            self.set_channel(p, key, False)
        self.panels.remove(p)
        self.panel_layout.removeWidget(p)
        p.map.setParent(None)
        p.deleteLater()
        self._relayout_maps()
        self._update_plot_visibility()

    def _relayout_maps(self):
        while self.map_grid.count():
            self.map_grid.takeAt(0)
        ncol = 2 if len(self.panels) <= 4 else 3
        for k, p in enumerate(self.panels):
            self.map_grid.addWidget(p.map, k // ncol, k % ncol)

    def refresh_ports(self, auto_assign: bool = False):
        ports = core.list_ports()
        for k, p in enumerate(self.panels):
            pref = ports[k].device if auto_assign and k < len(ports) else None
            if not p.is_active():
                p.set_ports(ports, pref)
        self.post_log("Ports: " + (", ".join(p.device for p in ports) or "none found"))

    def ports_in_use(self, exclude=None):
        return {p.worker.port for p in self.panels if p is not exclude and p.is_active()}

    def connect_all(self):
        for p in self.panels:
            if not p.is_active():
                p.toggle_connect()

    def disconnect_all(self):
        for p in self.panels:
            if p.is_active():
                p.disconnect_port()

    # ------------------------------------------------------------------
    # plot curves
    def set_channel(self, p: SensorPanel, key: str, on: bool):
        _k, label, _col, grp, color, _d = CH[key]
        plot = self.plots[grp].getPlotItem()
        if on and key not in p.curves:
            pen = pg.mkPen(color, width=1.6, style=LINE_STYLES[p.idx % len(LINE_STYLES)])
            c = pg.PlotDataItem(pen=pen, name=f"{p.tag} {label}", connect="finite")
            plot.addItem(c)
            p.curves[key] = c
        elif not on and key in p.curves:
            plot.removeItem(p.curves.pop(key))
        self._update_plot_visibility()

    def _update_plot_visibility(self):
        used = {CH[k][3] for p in self.panels for k in p.curves}
        for grp, pw in self.plots.items():
            pw.setVisible(grp in used or (not used and grp == "force"))

    # ------------------------------------------------------------------
    # recording
    def toggle_record(self, on: bool):
        if on:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            t0 = core.now()
            for p in self.panels:
                w = p.worker
                if w is None or w.state != "streaming":
                    continue
                fname = f"paxini_{ts}_{p.tag}_{w.port}_{w.model}.csv"
                path = os.path.join(RECORD_DIR, fname)
                try:
                    rec = core.CsvRecorder(path, w.n_points, t0)
                except OSError as e:
                    self.post_log(f"Cannot create {path}: {e}")
                    continue
                w.recorder = rec
                self.recorders.append((w, rec))
            if not self.recorders:
                QtWidgets.QMessageBox.warning(self, "Record", "No streaming sensor to record.")
                self.btn_rec.blockSignals(True)
                self.btn_rec.setChecked(False)
                self.btn_rec.blockSignals(False)
                return
            self.rec_t0 = t0
            self.btn_rec.setText("■  Stop & save CSV")
            self.btn_rec.setStyleSheet("QPushButton { background:#c62828; color:white;"
                                       " font-weight:bold; padding: 4px 10px; }")
            self.post_log(f"Recording started ({len(self.recorders)} sensor(s))")
        else:
            self._stop_recording()

    def _stop_recording(self):
        recs, self.recorders = self.recorders, []
        for m, _r in recs:
            m.recorder = None                  # detach first, then flush
        for m, r in recs:
            rows = r.stop()
            self.post_log(f"Saved {rows} rows -> {os.path.basename(r.path)}")
        self.btn_rec.setText("●  Start recording")
        self.btn_rec.setStyleSheet(self._rec_style_idle)
        self.rec_label.setText("Not recording")

    # ------------------------------------------------------------------
    # periodic GUI refresh (pulls data; workers never touch Qt)
    def on_tick(self):
        self._tick += 1
        self._drain_log()
        t_now = core.now()

        if self._tick % 4 == 0:
            for p in self.panels:
                p.refresh_status()
            if self.recorders:
                n = sum(r.rows for _m, r in self.recorders)
                self.rec_label.setText(f"REC {t_now - self.rec_t0:7.1f} s  ({n} rows written)")
        if self._tick % 3 == 0:
            for p in self.panels:
                p.refresh_values()

        if self.tabs.currentIndex() == 0:
            if not self.chk_pause.isChecked():
                win = self.spin_window.value()
                t_from = t_now - win
                for p in self.panels:
                    if not p.curves:
                        continue
                    m = p.module()
                    if m is None:
                        continue
                    keys = list(p.curves)
                    data = m.ring.window(t_from, [0] + [CH[k][2] for k in keys])
                    x = data[:, 0]
                    for j, k in enumerate(keys):
                        p.curves[k].setData(x, data[:, j + 1])
                for pw in self.plots.values():
                    if pw.isVisible():
                        pw.setXRange(t_from, t_now, padding=0)
                        break
        else:
            fs = self.spin_fs.value()
            ar = self.spin_arrow.value()
            for p in self.panels:
                m = p.module()
                if m is not None and m.latest_row is not None:
                    if p.map.xy is None or len(p.map.xy) != m.n_points:
                        p.map.set_coords(m.xy)
                    p.map.update_map(m.latest_taxels, m.latest_row, fs, ar)

    def closeEvent(self, ev):
        self.timer.stop()
        if self.recorders:
            self._stop_recording()
        for p in self.panels:
            p.disconnect_port()
        for p in self.panels:
            if p.worker is not None:
                p.worker.join(timeout=1.5)
        super().closeEvent(ev)


def main():
    pg.setConfigOptions(antialias=False, useOpenGL=False)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()



