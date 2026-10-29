"""Central import-path wiring for the unified GUI.

The motor client (``tue_usb``) and the PaXini sensor core
(``paxini_usb_core``) live in two plain directories beside the project root
rather than in installed packages.  This module makes both importable so the
rest of the application can simply ``import tue_usb`` / ``import
paxini_usb_core``.

It is imported first (from ``unified_gui/__init__``) which keeps a single,
predictable resolution order and prevents circular imports between the motor
and sensor subsystems.
"""
from __future__ import annotations

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SUBDIRS = ("python_reference", "Sensor")


def install() -> None:
    for name in SUBDIRS:
        path = os.path.join(REPO_ROOT, name)
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)


install()
