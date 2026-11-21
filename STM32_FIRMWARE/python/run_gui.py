"""
Launcher script for the TUE USB Binary Protocol PyQt6 GUI.

Examples:
    python run_gui.py                     # normal interactive session
    python run_gui.py --tab ps2           # open on the PS2 tab
    python run_gui.py --demo              # mock mode + PS2 tab (no hardware)
    python run_gui.py --mock --tab adc    # mock mode, ADC tab

`--demo` is the quick PS2 demonstration: the client connects to the built-in
mock engine, so the live 10 Hz 0x506 view shows simulated sticks and buttons
without a receiver board attached.
"""

import argparse
import os
import sys

# Ensure the python folder is in the python path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from gui.app import run_app


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="TUE USB Binary Protocol control GUI")
    parser.add_argument("--mock", action="store_true",
                        help="connect in mock mode (no hardware required)")
    parser.add_argument("--tab", default=None,
                        help="tab to open, e.g. ps2, adc, servo, dc")
    parser.add_argument("--demo", action="store_true",
                        help="shortcut for --mock --tab ps2")
    args = parser.parse_args(argv)

    mock = args.mock or args.demo
    tab = "ps2" if args.demo else args.tab
    return run_app(mock=mock, initial_tab=tab)


if __name__ == "__main__":
    sys.exit(main())
