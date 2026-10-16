"""Unified PyQt6 control & acquisition GUI.

Integrates multi-axis stepper motion, the Feetech STS3215 gripper servo and
high-speed dual PaXini tactile acquisition into one multi-threaded interface.

Importing this package wires the ``python_reference`` and ``Sensor``
directories onto ``sys.path`` so ``tue_usb`` and ``paxini_usb_core`` are
importable by the sub-modules.
"""
from __future__ import annotations

from . import _paths  # noqa: F401  (side effect: sys.path wiring)

__version__ = "1.0.0"
