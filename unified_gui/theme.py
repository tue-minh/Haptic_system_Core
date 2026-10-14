"""Light neutral theme (Phase 6.2).

The UI chrome is deliberately monochrome - light greys, one text colour and a
slightly inset field colour.  Colour is reserved for three things that actually
carry information:

* the four connection status LEDs (red / amber / green / orange), because
  link state is something you read at a glance;
* the stop/destructive actions (E-stop, Stop Recording), for safety
  convention;
* the 3D taxel colormap, which encodes normal-force magnitude.

Everything else distinguishes items by lightness and line style instead of hue:
sensor curves use a dark/mid grey pair plus solid/dashed/dotted pens, so a
monochrome plot still separates Left/Right and Fz/Fx/Fy.
"""
from __future__ import annotations

# --------------------------------------------------------------------------
# Surfaces and type
# --------------------------------------------------------------------------
BG = "#f0f1f4"            # window background
PANEL = "#ffffff"         # group box surface
PANEL_ALT = "#e9ebef"     # secondary surface (buttons, tabs)
FIELD = "#f7f8fa"         # inset readouts and inputs
BORDER = "#c9cdd5"
BORDER_HOVER = "#9aa0ac"
TEXT = "#1b1e23"
TEXT_DIM = "#5c626d"
TEXT_FAINT = "#9ba1ac"

# Interaction states
HOVER = "#dfe2e7"
PRESSED = "#d2d6dd"
DISABLED_BG = "#f3f4f6"
DISABLED_BORDER = "#e2e4e8"
SELECTION = "#c9cdd5"
SCROLLBAR = "#c2c7cf"
TOOLTIP_BG = "#ffffff"

# --------------------------------------------------------------------------
# Stop / destructive actions only
# --------------------------------------------------------------------------
DANGER = "#b23c3c"
DANGER_HOVER = "#c44a4a"
DANGER_BORDER = "#8f2f2f"
DANGER_TEXT = "#ffffff"

# Emphasised (non-destructive) action, e.g. Connect All / Start Recording.
PRIMARY = "#3c4046"
PRIMARY_HOVER = "#4a4f57"
PRIMARY_TEXT = "#ffffff"

# --------------------------------------------------------------------------
# Monochrome ramps
# --------------------------------------------------------------------------
# Sensor identity by lightness (Left dark, Right mid grey - both read against
# the light plot background); the pen style encodes the force direction
# (see panels/sensor.py).
SENSOR_SHADE = {"L": "#22262c", "R": "#8a919c"}

# --------------------------------------------------------------------------
# Connection status LEDs (four distinguishable states)
# --------------------------------------------------------------------------
LED_OFF = "#c0392b"         # disconnected
LED_CONNECTING = "#d09a1c"  # connecting
LED_READY = "#2e9e4f"       # streaming / ready
LED_ERROR = "#d2691e"       # error

# Pressure encoding for the 3D taxel map: greyscale, so idle taxels sit light
# against the panel and pressure darkens them. Full-hue maps stay selectable.
TAXEL_CMAP = "Greys"


def led_qss(color: str) -> str:
    """Stylesheet for a round LED indicator of the given colour."""
    return (
        f"background-color: {color}; border: 1px solid {BORDER_HOVER};"
        " border-radius: 8px; min-width: 16px; max-width: 16px;"
        " min-height: 16px; max-height: 16px;"
    )


STYLESHEET = f"""
QWidget {{
    background-color: {BG};
    color: {TEXT};
    font-family: "Segoe UI", "Noto Sans", sans-serif;
    font-size: 12px;
}}
QMainWindow, QDialog {{ background-color: {BG}; }}

QGroupBox {{
    background-color: {PANEL};
    border: 1px solid {BORDER};
    border-radius: 5px;
    margin-top: 14px;
    padding: 10px 8px 8px 8px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    color: {TEXT_DIM};
}}

QLabel {{ background: transparent; }}
QLabel:disabled {{ color: {TEXT_FAINT}; }}

QPushButton {{
    background-color: {PANEL_ALT};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 5px 12px;
    color: {TEXT};
}}
QPushButton:hover {{ background-color: {HOVER}; border-color: {BORDER_HOVER}; }}
QPushButton:pressed {{ background-color: {PRESSED}; }}
QPushButton:disabled {{
    color: {TEXT_FAINT}; background-color: {DISABLED_BG}; border-color: {DISABLED_BORDER};
}}

/* emphasised (non-destructive) action */
QPushButton#primary {{
    background-color: {PRIMARY};
    border: 1px solid {PRIMARY};
    color: {PRIMARY_TEXT};
    font-weight: bold;
}}
QPushButton#primary:hover {{ background-color: {PRIMARY_HOVER}; border-color: {PRIMARY_HOVER}; }}

/* stop / destructive action */
QPushButton#danger {{
    background-color: {DANGER};
    border: 1px solid {DANGER_BORDER};
    color: {DANGER_TEXT};
    font-weight: bold;
}}
QPushButton#danger:hover {{ background-color: {DANGER_HOVER}; }}

QPushButton#jog {{
    font-weight: bold;
    min-width: 46px;
    padding: 8px 6px;
}}

QComboBox, QDoubleSpinBox, QSpinBox, QLineEdit {{
    background-color: {FIELD};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 3px 6px;
    color: {TEXT};
    selection-background-color: {SELECTION};
}}
QComboBox:disabled, QDoubleSpinBox:disabled, QSpinBox:disabled, QLineEdit:disabled {{
    color: {TEXT_FAINT}; background-color: {DISABLED_BG};
}}
QComboBox::drop-down {{ border: none; width: 16px; }}
QComboBox QAbstractItemView {{
    background-color: {FIELD};
    border: 1px solid {BORDER};
    selection-background-color: {SELECTION};
}}

QSlider::groove:horizontal {{
    height: 5px; background: {BORDER}; border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {TEXT_DIM}; width: 14px; margin: -5px 0; border-radius: 7px;
}}
QSlider::handle:horizontal:disabled {{ background: {TEXT_FAINT}; }}

QCheckBox, QRadioButton {{ spacing: 6px; background: transparent; }}
QCheckBox::indicator, QRadioButton::indicator {{ width: 14px; height: 14px; }}
QCheckBox:disabled, QRadioButton:disabled {{ color: {TEXT_FAINT}; }}

QTabWidget::pane {{ border: 1px solid {BORDER}; border-radius: 4px; top: -1px; }}
QTabBar::tab {{
    background: {PANEL_ALT}; border: 1px solid {BORDER}; border-bottom: none;
    padding: 6px 14px; margin-right: 2px;
    border-top-left-radius: 4px; border-top-right-radius: 4px;
}}
QTabBar::tab:selected {{ background: {PANEL}; color: {TEXT}; font-weight: bold; }}

QSplitter::handle {{ background-color: {BORDER}; }}
QSplitter::handle:horizontal {{ width: 3px; }}
QSplitter::handle:vertical {{ height: 3px; }}

QScrollArea {{ border: none; }}

QPlainTextEdit, QTextEdit {{
    background-color: {FIELD};
    border: 1px solid {BORDER};
    border-radius: 4px;
    color: {TEXT_DIM};
}}

QTableWidget {{
    background-color: {PANEL};
    gridline-color: {BORDER};
    border: 1px solid {BORDER};
    border-radius: 4px;
}}
QHeaderView::section {{
    background-color: {PANEL_ALT};
    color: {TEXT_DIM};
    border: none;
    border-right: 1px solid {BORDER};
    border-bottom: 1px solid {BORDER};
    padding: 4px 6px;
    font-weight: bold;
}}

QStatusBar {{ background-color: {PANEL}; color: {TEXT_DIM}; border-top: 1px solid {BORDER}; }}

QToolTip {{
    background-color: {TOOLTIP_BG};
    color: {TEXT};
    border: 1px solid {BORDER_HOVER};
    padding: 3px;
}}

QScrollBar:vertical {{ background: {BG}; width: 11px; margin: 0; }}
QScrollBar::handle:vertical {{ background: {SCROLLBAR}; border-radius: 5px; min-height: 24px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar:horizontal {{ background: {BG}; height: 11px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: {SCROLLBAR}; border-radius: 5px; min-width: 24px; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}
"""

VALUE_QSS = (
    f"background-color: {FIELD}; border: 1px solid {BORDER}; border-radius: 4px;"
    f" padding: 2px 6px; color: {TEXT}; font-family: Consolas, monospace;"
)
