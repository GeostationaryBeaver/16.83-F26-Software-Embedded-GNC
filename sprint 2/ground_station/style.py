"""
Look of the ground station, matching index.html: monochrome instrument style, black on
white, 1px rules, mono labels. Colour only means state.
"""

from PySide6.QtGui import QFont

# Same tokens as index.html :root
BG = "#ffffff"
SUNK = "#f5f5f5"
INK = "#000000"
HAIR = "#d9d9d9"
GRID = "#ececec"
DIM = "#707070"
FAINT = "#a8a8a8"
BAR = "#000000"
BAR_TEXT = "#ffffff"
BAR_DIM = "#8c8c8c"
BAR_RULE = "#333333"
OK = "#0f8a4f"
BAD = "#e0281e"
WARN = "#d98a00"

MONO = ["JetBrains Mono", "SF Mono", "Menlo", "monospace"]
MONO_LIGHT = ["JetBrainsMono Nerd Font", "JetBrains Mono", "SF Mono", "Menlo"]  # has a weight-300 face
SANS = ["Helvetica Neue", "Helvetica", "Arial"]  # Inter, as in index.html, if installed: put it first

# State badge colours: (background, text)
STATE_BADGE = {"SAFE": (OK, "#fff"), "ARMED": (WARN, "#000"), "FAILSAFE": (BAD, "#fff")}


def font(size, weight=QFont.Normal, mono=True, spacing=0.0, families=None):
    """spacing: extra letter spacing in em, like CSS letter-spacing."""
    f = QFont()
    f.setFamilies(families or (MONO if mono else SANS))
    f.setPixelSize(size)
    f.setWeight(weight)
    if spacing:
        f.setLetterSpacing(QFont.AbsoluteSpacing, spacing * size)
    f.setHintingPreference(QFont.PreferNoHinting)
    return f


LABEL = dict(size=10, weight=QFont.Medium, spacing=0.06)  # the .label class: mono, uppercase, dim


def label_font():
    return font(**LABEL)


def set_variant(widget, variant):
    """Switches a button between the QSS variants below (estop, armed, ...)."""
    if widget.property("variant") != variant:
        widget.setProperty("variant", variant)
        widget.style().unpolish(widget)
        widget.style().polish(widget)


QSS = f"""
QMainWindow, QWidget#root, QWidget#side, QWidget#plots {{ background: {BG}; }}
QLabel {{ background: transparent; color: {INK}; }}

/* Top bar */
QFrame#bar {{ background: {BAR}; }}
QFrame#bar QLabel {{ color: {BAR_TEXT}; }}
QFrame#bar QLabel[role="label"] {{ color: {BAR_DIM}; }}
QFrame#cell, QFrame#brand {{ background: {BAR}; border: none; border-right: 1px solid {BAR_RULE}; }}
QFrame#conn {{ background: {BAR}; border: none; border-left: 1px solid {BAR_RULE}; }}

/* Buttons: square, ruled, invert on hover */
QPushButton {{
  font-family: "JetBrains Mono", "SF Mono", Menlo; font-size: 11px; font-weight: 500;
  color: {INK}; background: {BG}; border: 1px solid {INK}; border-radius: 0; padding: 8px 12px;
}}
QPushButton:enabled:hover {{ background: {INK}; color: {BG}; }}
QPushButton:disabled {{ color: {FAINT}; border-color: {HAIR}; }}
QPushButton[size="big"] {{ padding: 12px; }}
QPushButton[variant="armed"], QPushButton[variant="armed"]:enabled:hover {{
  background: {WARN}; border-color: {WARN}; color: #000; }}
QPushButton[variant="estop"] {{ border-color: {BAD}; color: {BAD}; font-weight: 600; }}
QPushButton[variant="estop"]:enabled:hover, QPushButton[variant="estop-on"],
QPushButton[variant="estop-on"]:enabled:hover {{
  background: {BAD}; border-color: {BAD}; color: #fff; font-weight: 600; }}
QFrame#bar QPushButton {{ background: {BAR}; color: {BAR_TEXT}; border-color: {BAR_TEXT}; }}
QFrame#bar QPushButton:enabled:hover {{ background: {BAR_TEXT}; color: {BAR}; }}
QFrame#bar QPushButton:disabled {{ color: {BAR_DIM}; border-color: {BAR_RULE}; }}
QFrame#bar QPushButton[variant="primary"] {{ background: {BAR_TEXT}; color: {BAR}; }}
QFrame#bar QPushButton[variant="primary"]:enabled:hover {{ background: {BAR}; color: {BAR_TEXT}; }}
QFrame#bar QPushButton[variant="rec-on"], QFrame#bar QPushButton[variant="rec-on"]:enabled:hover {{
  background: {BAD}; border-color: {BAD}; color: #fff; }}

/* Inputs */
QSpinBox, QComboBox {{
  font-family: "JetBrains Mono", "SF Mono", Menlo; font-size: 12px; color: {INK};
  background: {BG}; border: 1px solid {INK}; border-radius: 0; padding: 6px 8px;
}}
QSpinBox:disabled, QComboBox:disabled {{ color: {FAINT}; border-color: {HAIR}; }}
/* Hide the arrows here, not with NoButtons: with a style sheet, NoButtons collapses the editor to 1 px */
QSpinBox::up-button, QSpinBox::down-button {{ width: 0; height: 0; border: none; }}
QComboBox::drop-down {{ border: none; width: 18px; }}
QComboBox QAbstractItemView {{
  background: {BG}; color: {INK}; border: 1px solid {INK}; outline: none;
  selection-background-color: {INK}; selection-color: {BG};
}}
QFrame#bar QComboBox {{ background: {BAR}; color: {BAR_TEXT}; border: 1px solid {BAR_RULE}; }}
QFrame#bar QComboBox:disabled {{ color: {BAR_DIM}; }}
QComboBox#channel {{
  border: none; padding: 2px 0; background: transparent; color: {DIM};
  font-size: 10px; font-weight: 500;
}}
QComboBox#channel:hover {{ color: {INK}; }}

QSlider::groove:horizontal {{ height: 2px; background: {INK}; }}
QSlider::groove:horizontal:disabled {{ background: {HAIR}; }}
QSlider::handle:horizontal {{ width: 14px; height: 14px; margin: -6px 0; border-radius: 7px; background: {INK}; }}
QSlider::handle:horizontal:disabled {{ background: {HAIR}; }}

/* Layout: cells separated by 1px black rules */
QFrame#plotcell {{ background: {BG}; border: none; }}
QFrame#plotcell[right="1"] {{ border-right: 1px solid {INK}; }}
QFrame#plotcell[bottom="1"] {{ border-bottom: 1px solid {INK}; }}
QFrame#plotcell[right="1"][bottom="1"] {{ border-right: 1px solid {INK}; border-bottom: 1px solid {INK}; }}
QScrollArea#sidescroll {{ background: {BG}; border: none; border-left: 1px solid {INK}; }}
QFrame#section {{ background: {BG}; border: none; border-bottom: 1px solid {INK}; }}
QLabel#sectionhead {{ background: {SUNK}; color: {DIM}; border-bottom: 1px solid {HAIR}; padding: 8px 16px; }}
QLabel#notice {{ background: {BAD}; color: #fff; padding: 10px 16px; }}

QLabel[role="hint"] {{ color: {DIM}; }}
QLabel[role="dim"] {{ color: {DIM}; }}
QLabel[role="k"] {{ color: {DIM}; border-bottom: 1px solid {GRID}; padding: 4px 12px 4px 0; }}
QLabel[role="v"] {{ border-bottom: 1px solid {GRID}; padding: 4px 0; }}
QLabel[role="group"] {{ color: {DIM}; border-bottom: 1px solid {INK}; padding: 12px 0 5px 0; }}

QListWidget#log {{ border: none; background: {BG}; outline: none; }}
QListWidget#log::item {{ border-bottom: 1px solid {GRID}; padding: 0; }}
QListWidget#log::item:selected {{ background: {SUNK}; }}

QScrollBar:vertical {{ width: 8px; background: transparent; margin: 0; }}
QScrollBar::handle:vertical {{ background: {HAIR}; min-height: 30px; }}
QScrollBar::handle:vertical:hover {{ background: {FAINT}; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}
QToolTip {{ background: {INK}; color: {BG}; border: none; padding: 4px 6px; }}
"""
