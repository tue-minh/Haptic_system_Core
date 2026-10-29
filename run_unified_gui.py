"""Launcher for the Unified Motor Control & Dual PaXini Tactile GUI.

Run:  python run_unified_gui.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from unified_gui.app import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
