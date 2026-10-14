"""Application entry point for the unified control GUI.

Any of these work::

    python run_unified_gui.py          # launcher at the project root
    python -m unified_gui.app          # run as a module
    python unified_gui/app.py          # run the file directly
"""
from __future__ import annotations

import os
import sys

if __package__ in (None, ""):
    # Executed as a plain script: the package imports below need the project
    # root on sys.path, so hand off to the module entry point.
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from unified_gui.app import main as _main

    raise SystemExit(_main())

from . import theme


def _configure_rendering() -> None:
    """Pin the Matplotlib backend and tune pyqtgraph before any widget exists."""
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "1")
    try:
        import matplotlib

        matplotlib.use("QtAgg")
    except Exception:  # pragma: no cover - deps.verify() reports it
        pass
    try:
        import pyqtgraph as pg

        pg.setConfigOptions(antialias=False, useOpenGL=False, background=theme.BG,
                            foreground=theme.TEXT)
    except Exception:  # pragma: no cover
        pass


def main(argv=None) -> int:
    _configure_rendering()

    from PyQt6 import QtWidgets

    from .deps import verify, format_report

    report = verify()
    if not report.ok:
        app = QtWidgets.QApplication(sys.argv if argv is None else argv)
        QtWidgets.QMessageBox.critical(
            None, "Missing dependencies",
            format_report(report) + "\n\nInstall them with:\n    pip install -r requirements.txt",
        )
        return 2

    from .main_window import MainWindow

    app = QtWidgets.QApplication(sys.argv if argv is None else argv)
    app.setApplicationName("Unified Control & PaXini Tactile GUI")
    app.setStyle("Fusion")
    app.setStyleSheet(theme.STYLESHEET)

    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
