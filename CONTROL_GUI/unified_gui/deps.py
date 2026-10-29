"""Environment / dependency verification (Phase 1.1).

Checks the packages the unified GUI needs, validates the Matplotlib QtAgg
backend integration used by the embedded 3D tactile canvas, and reports
whether hardware-accelerated OpenGL rendering is available.

Run standalone for a pre-flight report::

    python -m unified_gui.deps
"""
from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from typing import List

# (import name, pip name, purpose)
REQUIRED = [
    ("PyQt6", "PyQt6", "GUI toolkit"),
    ("pyqtgraph", "pyqtgraph", "real-time 2D plotting"),
    ("serial", "pyserial", "COM-port I/O (motor + sensors)"),
    ("numpy", "numpy", "numerics / ring buffers"),
    ("matplotlib", "matplotlib", "embedded 3D tactile view"),
]


@dataclass
class Check:
    module: str
    package: str
    purpose: str
    ok: bool
    version: str = ""
    detail: str = ""


@dataclass
class DepReport:
    checks: List[Check] = field(default_factory=list)
    qtagg_ok: bool = False
    opengl_ok: bool = False
    notes: List[str] = field(default_factory=list)

    @property
    def missing(self) -> List[Check]:
        return [c for c in self.checks if not c.ok]

    @property
    def ok(self) -> bool:
        return not self.missing and self.qtagg_ok


def _version(mod) -> str:
    for attr in ("__version__", "version"):
        v = getattr(mod, attr, None)
        if isinstance(v, str) and v:
            return v
    try:  # PyQt6 exposes the version through QtCore
        from PyQt6.QtCore import PYQT_VERSION_STR  # noqa: PLC0415

        return PYQT_VERSION_STR
    except Exception:
        return ""


def verify() -> DepReport:
    """Import every required package and probe the optional backends."""
    report = DepReport()

    for module, package, purpose in REQUIRED:
        try:
            mod = importlib.import_module(module)
            report.checks.append(
                Check(module, package, purpose, True, version=_version(mod))
            )
        except Exception as exc:  # pragma: no cover - depends on env
            report.checks.append(
                Check(module, package, purpose, False, detail=str(exc))
            )

    # Matplotlib QtAgg backend (FigureCanvasQTAgg is what the 3D view embeds).
    try:
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg  # noqa: F401, PLC0415

        report.qtagg_ok = True
    except Exception as exc:  # pragma: no cover - depends on env
        report.qtagg_ok = False
        report.notes.append(f"Matplotlib QtAgg backend unavailable: {exc}")

    # Optional OpenGL acceleration (pyqtgraph.opengl -> PyOpenGL).
    try:
        importlib.import_module("OpenGL")
        importlib.import_module("pyqtgraph.opengl")

        report.opengl_ok = True
    except Exception:
        report.opengl_ok = False
        report.notes.append(
            "OpenGL (PyOpenGL / pyqtgraph.opengl) not available - the 3D tactile "
            "view will use the Matplotlib QtAgg fallback, which is fully "
            "functional but CPU-rendered.  Install with: pip install PyOpenGL"
        )

    return report


def format_report(report: DepReport) -> str:
    lines = ["Unified GUI - dependency check", "=" * 46]
    for c in report.checks:
        mark = "OK  " if c.ok else "MISS"
        ver = f" {c.version}" if c.ok and c.version else ""
        lines.append(f"[{mark}] {c.package:<12}{ver:<12} {c.purpose}")
        if not c.ok:
            lines.append(f"        -> {c.detail}")
    lines.append(f"[{'OK  ' if report.qtagg_ok else 'MISS'}] matplotlib QtAgg backend")
    lines.append(f"[{'OK  ' if report.opengl_ok else 'off '}] OpenGL hardware acceleration (optional)")
    if report.notes:
        lines.append("-" * 46)
        lines.extend(f"note: {n}" for n in report.notes)
    lines.append("=" * 46)
    lines.append("Result: " + ("READY" if report.ok else "MISSING REQUIRED PACKAGES"))
    return "\n".join(lines)


def main() -> int:
    report = verify()
    print(format_report(report))
    if not report.ok:
        print("\nInstall the missing packages, e.g.:")
        print("    pip install -r requirements.txt")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
