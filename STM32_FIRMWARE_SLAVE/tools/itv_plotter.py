"""Real-time 5-channel ITV pneumatic valve ADC plotter for STM32_FIRMWARE_SLAVE.

Connects to USART1 (115200 8N1), parses lines of the form:
    [t=1234ms] RAW:1024,2048,512,4095,0 | mV:825,1650,413,3300,0
and plots all 5 channels in real time using PyQt5 + pyqtgraph.
"""

from __future__ import annotations

import csv
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import pyqtgraph as pg
import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtGui, QtWidgets

LINE_RE = re.compile(
    r"\[t=(\d+)ms\]\s*RAW:(\d+),(\d+),(\d+),(\d+),(\d+)\s*\|\s*mV:(\d+),(\d+),(\d+),(\d+),(\d+)"
)

CH_COLORS = ["#00E5FF", "#FF5252", "#69F0AE", "#FFD740", "#E040FB"]
CH_NAMES = [f"CH{i} (PA{i})" for i in range(5)]
MAX_BUFFER_POINTS = 20000


@dataclass(slots=True)
class Sample:
    mcu_ms: int
    host_s: float
    raw: tuple[int, int, int, int, int]
    mv: tuple[int, int, int, int, int]


def parse_sample_line(line: str, host_s: float) -> Optional[Sample]:
    """Parse one telemetry line from STM32_FIRMWARE_SLAVE."""
    match = LINE_RE.search(line)
    if not match:
        return None
    nums = [int(g) for g in match.groups()]
    return Sample(
        mcu_ms=nums[0],
        host_s=host_s,
        raw=(nums[1], nums[2], nums[3], nums[4], nums[5]),
        mv=(nums[6], nums[7], nums[8], nums[9], nums[10]),
    )


class SerialWorker(QtCore.QObject):
    """Background serial reader + thread-safe command writer."""

    line_received = QtCore.pyqtSignal(str)
    status_changed = QtCore.pyqtSignal(bool, str)

    def __init__(self) -> None:
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._write_lock = threading.Lock()
        self._data_lock = threading.Lock()
        self.samples: deque[Sample] = deque(maxlen=MAX_BUFFER_POINTS)
        self._t0_mcu: Optional[int] = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect_port(self, port: str, baud: int) -> None:
        self.disconnect_port()
        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            self._ser.reset_input_buffer()
        except Exception as exc:
            self._ser = None
            self.status_changed.emit(False, f"Open {port} failed: {exc}")
            return

        self._t0_mcu = None
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.status_changed.emit(True, f"Connected to {port} @ {baud}")

    def disconnect_port(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.status_changed.emit(False, "Disconnected")

    def send_command(self, cmd: str) -> None:
        if not self.is_connected or self._ser is None:
            return
        payload = (cmd.strip() + "\n").encode("ascii", errors="ignore")
        with self._write_lock:
            try:
                self._ser.write(payload)
                self._ser.flush()
            except Exception as exc:
                self.line_received.emit(f"[GUI] TX error: {exc}")

    def clear_samples(self) -> None:
        with self._data_lock:
            self.samples.clear()
            self._t0_mcu = None

    def snapshot_samples(self) -> list[Sample]:
        with self._data_lock:
            return list(self.samples)

    def _read_loop(self) -> None:
        assert self._ser is not None
        t_start = time.monotonic()
        while not self._stop_event.is_set():
            try:
                raw_bytes = self._ser.readline()
            except Exception as exc:
                self.line_received.emit(f"[GUI] RX error: {exc}")
                break
            if not raw_bytes:
                continue

            text = raw_bytes.decode("utf-8", errors="replace").strip()
            if not text:
                continue

            sample = parse_sample_line(text, time.monotonic() - t_start)
            if sample is not None:
                with self._data_lock:
                    if self._t0_mcu is None:
                        self._t0_mcu = sample.mcu_ms
                    # Keep monotonically increasing time even if MCU resets
                    rel_s = (sample.mcu_ms - self._t0_mcu) / 1000.0
                    if rel_s < 0.0:
                        self._t0_mcu = sample.mcu_ms
                        self.samples.clear()
                        rel_s = 0.0
                    sample.host_s = rel_s
                    self.samples.append(sample)
            else:
                self.line_received.emit(text)


class ItvPlotterWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("STM32 ITV Valve 5-Channel ADC Plotter")
        self.resize(1220, 780)

        self.worker = SerialWorker()
        self.worker.line_received.connect(self._append_log)
        self.worker.status_changed.connect(self._on_status_changed)

        self._paused = False
        self._use_mv = True
        self._window_s = 15.0

        self._build_ui()
        self._refresh_ports()

        self._plot_timer = QtCore.QTimer(self)
        self._plot_timer.timeout.connect(self._update_plot)
        self._plot_timer.start(33)  # ~30 FPS UI refresh

    def _build_ui(self) -> None:
        pg.setConfigOptions(antialias=True, background="#121417", foreground="#E0E4EA")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # --- Row 1: Serial connection bar ---
        conn_bar = QtWidgets.QHBoxLayout()
        conn_bar.addWidget(QtWidgets.QLabel("Port:"))
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(190)
        conn_bar.addWidget(self.port_combo)

        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        conn_bar.addWidget(self.refresh_btn)

        conn_bar.addWidget(QtWidgets.QLabel("Baud:"))
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600"])
        conn_bar.addWidget(self.baud_combo)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.setCheckable(True)
        self.connect_btn.clicked.connect(self._toggle_connect)
        conn_bar.addWidget(self.connect_btn)

        conn_bar.addSpacing(16)
        self.status_lbl = QtWidgets.QLabel("Disconnected")
        self.status_lbl.setStyleSheet("color: #9AA4B2; font-weight: bold;")
        conn_bar.addWidget(self.status_lbl)
        conn_bar.addStretch()

        self.rate_hz_lbl = QtWidgets.QLabel("Samples: 0")
        conn_bar.addWidget(self.rate_hz_lbl)
        root.addLayout(conn_bar)

        # --- Row 2: Firmware & Plot controls ---
        ctrl_bar = QtWidgets.QHBoxLayout()
        for label, cmd in [
            ("Start", "start"),
            ("Stop", "stop"),
            ("Read Once", "read"),
            ("Stat", "stat"),
        ]:
            btn = QtWidgets.QPushButton(label)
            btn.clicked.connect(lambda _, c=cmd: self.worker.send_command(c))
            ctrl_bar.addWidget(btn)

        ctrl_bar.addSpacing(10)
        ctrl_bar.addWidget(QtWidgets.QLabel("Rate (ms):"))
        self.rate_spin = QtWidgets.QSpinBox()
        self.rate_spin.setRange(20, 10000)
        self.rate_spin.setSingleStep(10)
        self.rate_spin.setValue(50)
        ctrl_bar.addWidget(self.rate_spin)

        set_rate_btn = QtWidgets.QPushButton("Set Rate")
        set_rate_btn.clicked.connect(
            lambda: self.worker.send_command(f"rate {self.rate_spin.value()}")
        )
        ctrl_bar.addWidget(set_rate_btn)

        ctrl_bar.addSpacing(10)
        ctrl_bar.addWidget(QtWidgets.QLabel("Unit:"))
        self.unit_combo = QtWidgets.QComboBox()
        self.unit_combo.addItems(["mV (0-3300)", "RAW (0-4095)"])
        self.unit_combo.currentIndexChanged.connect(self._on_unit_changed)
        ctrl_bar.addWidget(self.unit_combo)

        ctrl_bar.addWidget(QtWidgets.QLabel("Window:"))
        self.win_combo = QtWidgets.QComboBox()
        self.win_combo.addItems(["5 s", "15 s", "30 s", "60 s", "All"])
        self.win_combo.setCurrentText("15 s")
        self.win_combo.currentTextChanged.connect(self._on_window_changed)
        ctrl_bar.addWidget(self.win_combo)

        self.autoscale_chk = QtWidgets.QCheckBox("Auto Y")
        self.autoscale_chk.setChecked(False)
        self.autoscale_chk.toggled.connect(self._apply_y_range)
        ctrl_bar.addWidget(self.autoscale_chk)

        ctrl_bar.addStretch()

        self.pause_btn = QtWidgets.QPushButton("Pause Plot")
        self.pause_btn.setCheckable(True)
        self.pause_btn.toggled.connect(self._on_pause_toggled)
        ctrl_bar.addWidget(self.pause_btn)

        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(self._clear_data)
        ctrl_bar.addWidget(clear_btn)

        save_btn = QtWidgets.QPushButton("Save CSV")
        save_btn.clicked.connect(self._save_csv)
        ctrl_bar.addWidget(save_btn)
        root.addLayout(ctrl_bar)

        # --- Row 3: Live channel cards ---
        ch_bar = QtWidgets.QHBoxLayout()
        self.ch_checks: list[QtWidgets.QCheckBox] = []
        self.ch_value_lbls: list[QtWidgets.QLabel] = []
        for i in range(5):
            box = QtWidgets.QFrame()
            box.setFrameShape(QtWidgets.QFrame.StyledPanel)
            box.setStyleSheet(
                f"QFrame {{ background: #1B1F26; border: 1px solid {CH_COLORS[i]}; border-radius: 6px; }}"
            )
            v = QtWidgets.QVBoxLayout(box)
            v.setContentsMargins(8, 6, 8, 6)
            v.setSpacing(2)

            chk = QtWidgets.QCheckBox(CH_NAMES[i])
            chk.setChecked(True)
            chk.setStyleSheet(f"color: {CH_COLORS[i]}; font-weight: bold;")
            chk.toggled.connect(self._update_visibility)
            v.addWidget(chk)
            self.ch_checks.append(chk)

            val_lbl = QtWidgets.QLabel("--- mV (----)")
            val_lbl.setStyleSheet("color: #EAEFF5; font-family: Consolas; font-size: 13px; border: none;")
            v.addWidget(val_lbl)
            self.ch_value_lbls.append(val_lbl)

            ch_bar.addWidget(box)
        root.addLayout(ch_bar)

        # --- Row 4: Real-time pyqtgraph Plot ---
        self.plot_widget = pg.PlotWidget(title="ITV Pneumatic Valve Feedback (5x ADC)")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.setLabel("left", "Voltage", units="mV")
        self.plot_widget.addLegend(offset=(10, 10))
        self.plot_widget.setYRange(0, 3300, padding=0.04)

        self.curves: list[pg.PlotDataItem] = []
        for i in range(5):
            pen = pg.mkPen(color=CH_COLORS[i], width=2)
            curve = self.plot_widget.plot([], [], name=CH_NAMES[i], pen=pen)
            self.curves.append(curve)

        root.addWidget(self.plot_widget, stretch=1)

        # --- Row 5: Console log + manual command input ---
        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(300)
        self.log_box.setFixedHeight(110)
        self.log_box.setStyleSheet(
            "background: #0E1013; color: #B8C2CC; font-family: Consolas; font-size: 11px;"
        )
        root.addWidget(self.log_box)

        cmd_bar = QtWidgets.QHBoxLayout()
        self.cmd_edit = QtWidgets.QLineEdit()
        self.cmd_edit.setPlaceholderText("Send USART1 command (e.g. help, rate 20, read, stat)...")
        self.cmd_edit.returnPressed.connect(self._send_manual_cmd)
        cmd_bar.addWidget(self.cmd_edit)

        send_btn = QtWidgets.QPushButton("Send")
        send_btn.clicked.connect(self._send_manual_cmd)
        cmd_bar.addWidget(send_btn)
        root.addLayout(cmd_bar)

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        self.port_combo.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        for p in ports:
            label = f"{p.device} — {p.description}" if p.description else p.device
            self.port_combo.addItem(label, p.device)
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def _toggle_connect(self, checked: bool) -> None:
        if checked:
            port = self.port_combo.currentData()
            if not port:
                self.connect_btn.setChecked(False)
                QtWidgets.QMessageBox.warning(self, "No Port", "No COM port selected.")
                return
            baud = int(self.baud_combo.currentText())
            self.worker.connect_port(port, baud)
        else:
            self.worker.disconnect_port()

    def _on_status_changed(self, connected: bool, message: str) -> None:
        self.connect_btn.setChecked(connected)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.status_lbl.setText(message)
        self.status_lbl.setStyleSheet(
            f"color: {'#69F0AE' if connected else '#FF5252'}; font-weight: bold;"
        )
        self._append_log(f"[GUI] {message}")

    def _on_unit_changed(self, idx: int) -> None:
        self._use_mv = idx == 0
        self.plot_widget.setLabel("left", "Voltage" if self._use_mv else "ADC Raw", units="mV" if self._use_mv else "cnt")
        self._apply_y_range()

    def _on_window_changed(self, text: str) -> None:
        if text == "All":
            self._window_s = 0.0
        else:
            self._window_s = float(text.split()[0])

    def _apply_y_range(self) -> None:
        if self.autoscale_chk.isChecked():
            self.plot_widget.enableAutoRange(axis="y", enable=True)
        else:
            self.plot_widget.enableAutoRange(axis="y", enable=False)
            self.plot_widget.setYRange(0, 3300 if self._use_mv else 4095, padding=0.04)

    def _on_pause_toggled(self, paused: bool) -> None:
        self._paused = paused
        self.pause_btn.setText("Resume Plot" if paused else "Pause Plot")

    def _update_visibility(self) -> None:
        for i, chk in enumerate(self.ch_checks):
            self.curves[i].setVisible(chk.isChecked())

    def _clear_data(self) -> None:
        self.worker.clear_samples()
        for curve in self.curves:
            curve.setData([], [])
        for lbl in self.ch_value_lbls:
            lbl.setText("--- mV (----)")
        self.rate_hz_lbl.setText("Samples: 0")

    def _send_manual_cmd(self) -> None:
        cmd = self.cmd_edit.text().strip()
        if cmd:
            self.worker.send_command(cmd)
            self.cmd_edit.clear()

    def _append_log(self, text: str) -> None:
        self.log_box.appendPlainText(text)

    def _update_plot(self) -> None:
        samples = self.worker.snapshot_samples()
        if not samples:
            return

        latest = samples[-1]
        for i in range(5):
            self.ch_value_lbls[i].setText(f"{latest.mv[i]:4d} mV ({latest.raw[i]:4d})")
        self.rate_hz_lbl.setText(f"Samples: {len(samples)}")

        if self._paused:
            return

        t_end = latest.host_s
        if self._window_s > 0.0:
            t_min = max(0.0, t_end - self._window_s)
            visible = [s for s in samples if s.host_s >= t_min]
        else:
            visible = samples

        if not visible:
            return

        xs = [s.host_s for s in visible]
        for i in range(5):
            if self.ch_checks[i].isChecked():
                ys = [s.mv[i] if self._use_mv else s.raw[i] for s in visible]
                self.curves[i].setData(xs, ys)

        if self._window_s > 0.0:
            self.plot_widget.setXRange(max(0.0, t_end - self._window_s), max(self._window_s, t_end), padding=0.01)
        else:
            self.plot_widget.enableAutoRange(axis="x", enable=True)

    def _save_csv(self) -> None:
        samples = self.worker.snapshot_samples()
        if not samples:
            QtWidgets.QMessageBox.information(self, "No Data", "No samples recorded yet.")
            return

        default_name = f"itv_adc_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        path_str, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save ADC Log", str(Path.cwd() / default_name), "CSV Files (*.csv)"
        )
        if not path_str:
            return

        with open(path_str, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(
                [
                    "mcu_ms",
                    "time_s",
                    "raw_ch0",
                    "raw_ch1",
                    "raw_ch2",
                    "raw_ch3",
                    "raw_ch4",
                    "mv_ch0",
                    "mv_ch1",
                    "mv_ch2",
                    "mv_ch3",
                    "mv_ch4",
                ]
            )
            for s in samples:
                writer.writerow([s.mcu_ms, f"{s.host_s:.3f}", *s.raw, *s.mv])
        self._append_log(f"[GUI] Saved {len(samples)} samples to {path_str}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.worker.disconnect_port()
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ItvPlotterWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
