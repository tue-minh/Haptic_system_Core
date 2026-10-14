"""Recording control panel (Phase 5.1, 5.4).

Start/Stop toggle with live elapsed-time / row-count / file-path feedback and a
configurable output directory (workspace or Desktop defaults).
"""
from __future__ import annotations

import os

from PyQt6 import QtWidgets
from PyQt6.QtCore import pyqtSignal

from .. import theme
from .._paths import REPO_ROOT
from ..recording import DEFAULT_RECORD_RATE_HZ

# Quick-pick output locations (Phase 5.4: workspace / desktop defaults).
WORKSPACE_DIR = os.path.join(REPO_ROOT, "Unified_Control")
DESKTOP_DIR = os.path.join(os.path.expanduser("~"), "Desktop", "Unified_Control")


def format_hms(seconds: float) -> str:
    seconds = max(0, int(seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{s:02d}"


class RecordingPanel(QtWidgets.QGroupBox):
    start_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    output_dir_changed = pyqtSignal(str)

    def __init__(self, output_dir: str, parent=None) -> None:
        super().__init__("Data Recording", parent)
        self._recording = False

        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(8)

        top = QtWidgets.QHBoxLayout()
        self.btn_record = QtWidgets.QPushButton("\u25cf  Start Recording")
        self.btn_record.setObjectName("primary")
        self.btn_record.setMinimumHeight(34)
        self.btn_record.clicked.connect(self._on_toggle)
        top.addWidget(self.btn_record, 2)

        self.lbl_elapsed = QtWidgets.QLabel("Elapsed 00:00:00")
        self.lbl_elapsed.setFont(QtWidgets.QApplication.font())
        self.lbl_elapsed.setStyleSheet(theme.VALUE_QSS)
        top.addWidget(self.lbl_elapsed, 1)
        root.addLayout(top)

        self.lbl_rows = QtWidgets.QLabel("Rows logged: 0")
        self.lbl_rows.setStyleSheet(f"color: {theme.TEXT_DIM};")
        root.addWidget(self.lbl_rows)

        self.lbl_files = QtWidgets.QLabel("No file yet")
        self.lbl_files.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.lbl_files.setWordWrap(True)
        root.addWidget(self.lbl_files)

        drow = QtWidgets.QHBoxLayout()
        drow.addWidget(QtWidgets.QLabel("Output"))
        self.edit_dir = QtWidgets.QLineEdit(output_dir)
        self.edit_dir.editingFinished.connect(self._on_dir_edited)
        drow.addWidget(self.edit_dir, 1)
        btn_ws = QtWidgets.QPushButton("Workspace")
        btn_ws.setToolTip("Recordings folder inside the project workspace")
        btn_ws.clicked.connect(lambda: self._set_dir(WORKSPACE_DIR))
        btn_desk = QtWidgets.QPushButton("Desktop")
        btn_desk.setToolTip("Record to the current user's Desktop")
        btn_desk.clicked.connect(lambda: self._set_dir(DESKTOP_DIR))
        btn_browse = QtWidgets.QPushButton("Browse…")
        btn_browse.clicked.connect(self._browse)
        drow.addWidget(btn_ws)
        drow.addWidget(btn_desk)
        drow.addWidget(btn_browse)
        root.addLayout(drow)

        rrow = QtWidgets.QHBoxLayout()
        rrow.addWidget(QtWidgets.QLabel("Master sample rate"))
        self.spin_rate = QtWidgets.QDoubleSpinBox()
        self.spin_rate.setRange(10.0, 500.0)
        self.spin_rate.setValue(DEFAULT_RECORD_RATE_HZ)
        self.spin_rate.setSuffix(" Hz")
        rrow.addWidget(self.spin_rate)
        rrow.addStretch(1)
        root.addLayout(rrow)

    # -- state --------------------------------------------------------------
    @property
    def recording(self) -> bool:
        return self._recording

    @property
    def output_dir(self) -> str:
        return self.edit_dir.text().strip()

    @property
    def rate_hz(self) -> float:
        return float(self.spin_rate.value())

    def set_recording(self, on: bool) -> None:
        self._recording = on
        if on:
            # recording active is a stop action, so it uses the single accent
            # colour the theme reserves for "press me to stop"
            self.btn_record.setText("\u25a0  Stop Recording")
            self.btn_record.setStyleSheet(
                f"QPushButton {{ background-color: {theme.DANGER}; color: {theme.DANGER_TEXT};"
                f" font-weight: bold; border: 1px solid {theme.DANGER_BORDER};"
                " border-radius: 4px; padding: 5px 12px; }"
            )
        else:
            self.btn_record.setText("\u25cf  Start Recording")
            self.btn_record.setStyleSheet("")
            self.btn_record.setObjectName("primary")
            self.btn_record.style().unpolish(self.btn_record)
            self.btn_record.style().polish(self.btn_record)

    def set_enabled(self, on: bool) -> None:
        self.btn_record.setEnabled(on)

    def update_status(self, elapsed: float = 0.0, rows: int = 0,
                      files=None, note: str = "") -> None:
        self.lbl_elapsed.setText(f"Elapsed {format_hms(elapsed)}")
        self.lbl_rows.setText(f"Rows logged: {rows}")
        if files:
            names = [os.path.basename(f) for f in files]
            shown = names if len(names) <= 3 else names[:3] + [f"+{len(names) - 3} more"]
            self.lbl_files.setText("Files: " + ", ".join(shown))
        elif note:
            self.lbl_files.setText(note)

    # -- actions ------------------------------------------------------------
    def _on_toggle(self) -> None:
        if self._recording:
            self.stop_requested.emit()
        else:
            self.start_requested.emit()

    def _on_dir_edited(self) -> None:
        self.output_dir_changed.emit(self.output_dir)

    def _set_dir(self, path: str) -> None:
        self.edit_dir.setText(path)
        self.output_dir_changed.emit(path)

    def _browse(self) -> None:
        chosen = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select recording output directory", self.output_dir or os.getcwd()
        )
        if chosen:
            self.edit_dir.setText(chosen)
            self.output_dir_changed.emit(chosen)
