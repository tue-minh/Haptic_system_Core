"""Real-time 5-channel ITV pneumatic valve ADC plotter for STM32_FIRMWARE_SLAVE."""

from __future__ import annotations

import csv
import queue
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque

import pyqtgraph as pg
import serial
import serial.tools.list_ports
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

LINE_RE = re.compile(
    r"\[t=(\d+)ms\]\s*RAW:(\d+),(\d+),(\d+),(\d+),(\d+)\s*\|\s*mV:(\d+),(\d+),(\d+),(\d+),(\d+)"
)

CH_COUNT = 5
CH_COLORS = ["#38bdf8", "#4ade80", "#facc15", "#fb7185", "#c084fc"]
CH_LABELS = [f"CH{i} (PA{i})" for i in range(CH_COUNT)]
MAX_HISTORY_POINTS = 6000  # e.g., 120 s at 50 Hz


@dataclass(frozen=True)
class Sample:
    host_time: float
    mcu_ms: int
    raw: tuple[int, ...]
    mv: tuple[int, ...]


def parse_telemetry_line(line: str, host_time: float | None = None) -> Sample | None:
    """Parse a firmware USART1 telemetry line into a Sample, or return None."""
    match = LINE_RE.search(line)
    if not match:
        return None
    nums = [int(g) for g in match.groups()]
    return Sample(
        host_time=time.monotonic() if host_time is None else host_time,
        mcu_ms=nums[0],
        raw=tuple(nums[1:6]),
        mv=tuple(nums[6:11]),
    )


def apply_channel_filter(values: list[int], mode: str) -> list[float]:
    """Filter a single channel's sample series using the selected smoothing mode."""
    if not values or mode == "None":
        return [float(v) for v in values]

    if mode.startswith("EMA"):
        alpha = 0.35 if "0.35" in mode else 0.15
        out: list[float] = [float(values[0])]
        prev = out[0]
        for v in values[1:]:
            prev = alpha * float(v) + (1.0 - alpha) * prev
            out.append(prev)
        return out

    win = 5 if "(5)" in mode else 10
    out = []
    running = 0.0
    for idx, v in enumerate(values):
        running += float(v)
        if idx >= win:
            running -= float(values[idx - win])
            out.append(running / win)
        else:
            out.append(running / (idx + 1))
    return out


class SerialWorker:
    """Background serial reader thread pushing parsed samples and log lines to queues."""

    def __init__(self, sample_q: queue.Queue[Sample], log_q: queue.Queue[str]) -> None:
        self._sample_q = sample_q
        self._log_q = log_q
        self._ser: serial.Serial | None = None
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        self._tx_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self, port: str, baud: int) -> None:
        self.disconnect()
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self._ser.reset_input_buffer()
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
            self._ser = None

    def send_command(self, cmd: str) -> None:
        if not self.is_connected or self._ser is None:
            return
        payload = (cmd.strip() + "\r\n").encode("ascii", errors="replace")
        with self._tx_lock:
            self._ser.write(payload)

    def _run(self) -> None:
        assert self._ser is not None
        while not self._stop_evt.is_set():
            try:
                raw_bytes = self._ser.readline()
            except serial.SerialException as exc:
                self._log_q.put(f"[SERIAL ERROR] {exc}")
                break

            if not raw_bytes:
                continue

            text = raw_bytes.decode("utf-8", errors="replace").strip()
            if not text:
                continue

            sample = parse_telemetry_line(text)
            if sample is not None:
                self._sample_q.put(sample)
            else:
                self._log_q.put(text)


class ItvPlotterWindow(QtWidgets.QMainWindow):
    """Main window for real-time 5-channel ITV valve telemetry plotting."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("STM32 ITV Valve 5-Ch Real-Time ADC Plotter")
        self.resize(1220, 780)

        self._sample_q: queue.Queue[Sample] = queue.Queue()
        self._log_q: queue.Queue[str] = queue.Queue()
        self._worker = SerialWorker(self._sample_q, self._log_q)

        self._t0_mcu_ms: int | None = None
        self._times: Deque[float] = deque(maxlen=MAX_HISTORY_POINTS)
        self._raw_hist: list[Deque[int]] = [
            deque(maxlen=MAX_HISTORY_POINTS) for _ in range(CH_COUNT)
        ]
        self._mv_hist: list[Deque[int]] = [
            deque(maxlen=MAX_HISTORY_POINTS) for _ in range(CH_COUNT)
        ]
        self._rate_timestamps: Deque[float] = deque(maxlen=100)

        self._paused = False
        self._csv_file = None
        self._csv_writer = None

        self._build_ui()
        self._refresh_ports()

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(25)  # ~40 FPS UI refresh
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()

    def _build_ui(self) -> None:
        pg.setConfigOptions(antialias=True)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # --- Connection & firmware control toolbar ---
        conn_row = QtWidgets.QHBoxLayout()
        root.addLayout(conn_row)

        conn_row.addWidget(QtWidgets.QLabel("Port:"))
        self._port_combo = QtWidgets.QComboBox()
        self._port_combo.setMinimumWidth(220)
        conn_row.addWidget(self._port_combo)

        self._refresh_btn = QtWidgets.QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        conn_row.addWidget(self._refresh_btn)

        conn_row.addWidget(QtWidgets.QLabel("Baud:"))
        self._baud_combo = QtWidgets.QComboBox()
        self._baud_combo.addItems(["115200", "230400", "460800", "921600"])
        conn_row.addWidget(self._baud_combo)

        self._connect_btn = QtWidgets.QPushButton("Connect")
        self._connect_btn.setCheckable(True)
        self._connect_btn.clicked.connect(self._toggle_connect)
        conn_row.addWidget(self._connect_btn)

        conn_row.addSpacing(12)

        for label, cmd in [("Start", "start"), ("Stop", "stop"), ("Read", "read"), ("Stat", "stat")]:
            btn = QtWidgets.QPushButton(label)
            btn.clicked.connect(lambda _=False, c=cmd: self._send_cmd(c))
            conn_row.addWidget(btn)

        conn_row.addWidget(QtWidgets.QLabel("Rate (ms):"))
        self._rate_spin = QtWidgets.QSpinBox()
        self._rate_spin.setRange(20, 10000)
        self._rate_spin.setSingleStep(10)
        self._rate_spin.setValue(50)
        conn_row.addWidget(self._rate_spin)

        set_rate_btn = QtWidgets.QPushButton("Set")
        set_rate_btn.clicked.connect(lambda: self._send_cmd(f"rate {self._rate_spin.value()}"))
        conn_row.addWidget(set_rate_btn)

        conn_row.addStretch()

        self._status_lbl = QtWidgets.QLabel("Disconnected")
        self._status_lbl.setStyleSheet("color: #f87171; font-weight: bold;")
        conn_row.addWidget(self._status_lbl)

        # --- Plot options row ---
        opt_row = QtWidgets.QHBoxLayout()
        root.addLayout(opt_row)

        opt_row.addWidget(QtWidgets.QLabel("Units:"))
        self._unit_combo = QtWidgets.QComboBox()
        self._unit_combo.addItems(["mV (0 - 3300)", "RAW (0 - 4095)"])
        self._unit_combo.currentIndexChanged.connect(self._on_unit_changed)
        opt_row.addWidget(self._unit_combo)

        opt_row.addWidget(QtWidgets.QLabel("Filter:"))
        self._filter_combo = QtWidgets.QComboBox()
        self._filter_combo.addItems(
            ["None", "EMA (α=0.35)", "EMA (α=0.15)", "Moving Avg (5)", "Moving Avg (10)"]
        )
        self._filter_combo.currentIndexChanged.connect(lambda _idx: self._redraw_curves())
        opt_row.addWidget(self._filter_combo)

        opt_row.addWidget(QtWidgets.QLabel("Window (s):"))
        self._window_spin = QtWidgets.QSpinBox()
        self._window_spin.setRange(2, 120)
        self._window_spin.setValue(15)
        opt_row.addWidget(self._window_spin)

        self._pause_btn = QtWidgets.QPushButton("Pause Plot")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        opt_row.addWidget(self._pause_btn)

        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(self._clear_plot)
        opt_row.addWidget(clear_btn)

        self._record_btn = QtWidgets.QPushButton("Record CSV")
        self._record_btn.setCheckable(True)
        self._record_btn.toggled.connect(self._toggle_csv_record)
        opt_row.addWidget(self._record_btn)

        opt_row.addStretch()

        # --- Channel readout cards ---
        cards_row = QtWidgets.QHBoxLayout()
        root.addLayout(cards_row)

        self._ch_checks: list[QtWidgets.QCheckBox] = []
        self._ch_readouts: list[QtWidgets.QLabel] = []
        for i in range(CH_COUNT):
            card = QtWidgets.QFrame()
            card.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
            card.setStyleSheet(
                f"QFrame {{ background: #1e293b; border: 1px solid {CH_COLORS[i]}; border-radius: 6px; }}"
            )
            card_layout = QtWidgets.QVBoxLayout(card)
            card_layout.setContentsMargins(8, 6, 8, 6)

            chk = QtWidgets.QCheckBox(CH_LABELS[i])
            chk.setChecked(True)
            chk.setStyleSheet(f"color: {CH_COLORS[i]}; font-weight: bold;")
            chk.toggled.connect(lambda checked, idx=i: self._curves[idx].setVisible(checked))
            card_layout.addWidget(chk)
            self._ch_checks.append(chk)

            val_lbl = QtWidgets.QLabel("0 mV  |  RAW: 0")
            val_lbl.setStyleSheet("color: #f8fafc; font-family: Consolas, monospace; font-size: 13px;")
            card_layout.addWidget(val_lbl)
            self._ch_readouts.append(val_lbl)

            cards_row.addWidget(card)

        # --- Real-time PyQtGraph plot ---
        self._plot = pg.PlotWidget()
        self._plot.setBackground("#0f172a")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("bottom", "Time", units="s")
        self._plot.setLabel("left", "Voltage", units="mV")
        self._plot.setYRange(-50, 3350, padding=0)
        self._plot.addLegend(offset=(10, 10))
        root.addWidget(self._plot, stretch=4)

        self._curves: list[pg.PlotDataItem] = []
        for i in range(CH_COUNT):
            pen = pg.mkPen(color=CH_COLORS[i], width=2)
            curve = self._plot.plot([], [], name=CH_LABELS[i], pen=pen)
            self._curves.append(curve)

        # --- Serial console ---
        console_box = QtWidgets.QGroupBox("USART1 Console")
        console_layout = QtWidgets.QVBoxLayout(console_box)
        root.addWidget(console_box, stretch=1)

        self._console = QtWidgets.QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setMaximumBlockCount(300)
        self._console.setFont(QtGui.QFont("Consolas", 9))
        console_layout.addWidget(self._console)

        cmd_row = QtWidgets.QHBoxLayout()
        self._cmd_edit = QtWidgets.QLineEdit()
        self._cmd_edit.setPlaceholderText("Send USART1 command (help, start, stop, read, rate 50, stat)")
        self._cmd_edit.returnPressed.connect(self._send_custom_cmd)
        cmd_row.addWidget(self._cmd_edit)

        send_btn = QtWidgets.QPushButton("Send")
        send_btn.clicked.connect(self._send_custom_cmd)
        cmd_row.addWidget(send_btn)
        console_layout.addLayout(cmd_row)

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentData()
        self._port_combo.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        for p in ports:
            self._port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if current is not None:
            idx = self._port_combo.findData(current)
            if idx >= 0:
                self._port_combo.setCurrentIndex(idx)

    def _toggle_connect(self, checked: bool) -> None:
        if checked:
            port = self._port_combo.currentData()
            if not port:
                self._connect_btn.setChecked(False)
                self._append_log("[GUI] No COM port selected.")
                return
            baud = int(self._baud_combo.currentText())
            try:
                self._worker.connect(port, baud)
            except serial.SerialException as exc:
                self._connect_btn.setChecked(False)
                self._append_log(f"[GUI] Failed to open {port}: {exc}")
                return
            self._connect_btn.setText("Disconnect")
            self._status_lbl.setText(f"Connected ({port})")
            self._status_lbl.setStyleSheet("color: #4ade80; font-weight: bold;")
            self._append_log(f"[GUI] Connected to {port} @ {baud} baud")
        else:
            self._worker.disconnect()
            self._connect_btn.setText("Connect")
            self._status_lbl.setText("Disconnected")
            self._status_lbl.setStyleSheet("color: #f87171; font-weight: bold;")
            self._append_log("[GUI] Disconnected")

    def _send_cmd(self, cmd: str) -> None:
        if not self._worker.is_connected:
            self._append_log("[GUI] Not connected.")
            return
        self._worker.send_command(cmd)
        self._append_log(f"> {cmd}")

    def _send_custom_cmd(self) -> None:
        cmd = self._cmd_edit.text().strip()
        if not cmd:
            return
        self._cmd_edit.clear()
        self._send_cmd(cmd)

    def _on_unit_changed(self, idx: int) -> None:
        if idx == 0:
            self._plot.setLabel("left", "Voltage", units="mV")
            self._plot.setYRange(-50, 3350, padding=0)
        else:
            self._plot.setLabel("left", "ADC Raw", units="counts")
            self._plot.setYRange(-50, 4150, padding=0)
        self._redraw_curves()

    def _on_pause_toggled(self, checked: bool) -> None:
        self._paused = checked
        self._pause_btn.setText("Resume Plot" if checked else "Pause Plot")

    def _clear_plot(self) -> None:
        self._t0_mcu_ms = None
        self._times.clear()
        self._rate_timestamps.clear()
        for i in range(CH_COUNT):
            self._raw_hist[i].clear()
            self._mv_hist[i].clear()
            self._curves[i].setData([], [])

    def _toggle_csv_record(self, checked: bool) -> None:
        if checked:
            out_dir = Path(__file__).resolve().parent / "logs"
            out_dir.mkdir(parents=True, exist_ok=True)
            fname = out_dir / f"itv_adc_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self._csv_file = open(fname, "w", newline="", encoding="utf-8")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(
                [
                    "mcu_ms",
                    "raw_ch0", "raw_ch1", "raw_ch2", "raw_ch3", "raw_ch4",
                    "mv_ch0", "mv_ch1", "mv_ch2", "mv_ch3", "mv_ch4",
                ]
            )
            self._record_btn.setText("Stop CSV")
            self._append_log(f"[GUI] Recording CSV to {fname}")
        else:
            self._close_csv()
            self._record_btn.setText("Record CSV")
            self._append_log("[GUI] CSV recording stopped")

    def _close_csv(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

    def _append_log(self, msg: str) -> None:
        self._console.appendPlainText(msg)

    def _on_tick(self) -> None:
        while not self._log_q.empty():
            self._append_log(self._log_q.get_nowait())

        latest: Sample | None = None
        got_new = False
        while not self._sample_q.empty():
            sample = self._sample_q.get_nowait()
            latest = sample
            got_new = True
            self._rate_timestamps.append(sample.host_time)

            if self._t0_mcu_ms is None or sample.mcu_ms < self._t0_mcu_ms:
                self._t0_mcu_ms = sample.mcu_ms
            t_sec = (sample.mcu_ms - self._t0_mcu_ms) / 1000.0

            self._times.append(t_sec)
            for i in range(CH_COUNT):
                self._raw_hist[i].append(sample.raw[i])
                self._mv_hist[i].append(sample.mv[i])

            if self._csv_writer is not None:
                self._csv_writer.writerow([sample.mcu_ms, *sample.raw, *sample.mv])

        if latest is not None:
            for i in range(CH_COUNT):
                self._ch_readouts[i].setText(f"{latest.mv[i]:4d} mV  |  RAW: {latest.raw[i]:4d}")

        if self._worker.is_connected and len(self._rate_timestamps) >= 2:
            dt = self._rate_timestamps[-1] - self._rate_timestamps[0]
            if dt > 0:
                hz = (len(self._rate_timestamps) - 1) / dt
                port = self._port_combo.currentData()
                self._status_lbl.setText(f"Connected ({port}) — {hz:.1f} Hz")

        if got_new and not self._paused:
            self._redraw_curves()

    def _redraw_curves(self) -> None:
        if not self._times:
            return
        t_list = list(self._times)
        t_max = t_list[-1]
        win = float(self._window_spin.value())
        t_min = max(0.0, t_max - win)
        self._plot.setXRange(t_min, max(win, t_max), padding=0.02)

        use_mv = self._unit_combo.currentIndex() == 0
        filter_mode = self._filter_combo.currentText()
        for i in range(CH_COUNT):
            mv_vals = apply_channel_filter(list(self._mv_hist[i]), filter_mode)
            raw_vals = apply_channel_filter(list(self._raw_hist[i]), filter_mode)
            self._curves[i].setData(t_list, mv_vals if use_mv else raw_vals)
            self._ch_readouts[i].setText(
                f"{round(mv_vals[-1]):4d} mV  |  RAW: {round(raw_vals[-1]):4d}"
            )

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._worker.disconnect()
        self._close_csv()
        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    win = ItvPlotterWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
