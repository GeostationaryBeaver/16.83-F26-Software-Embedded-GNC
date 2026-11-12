#!/usr/bin/env python3
"""
Hopper ground station: the dashboard as a desktop app.

Talks to the ground board (firmware-arduino/ground) over USB. The ground board relays
commands to the hopper over ESP-NOW and its telemetry back. Commands go out 50 times a
second from their own thread (link.py), so the window can be slow or minimised without
the hopper going FAILSAFE.

Run:  python ground_station.py [--port /dev/cu.usbmodem...]
Esc = E-stop. Closing the window disarms the hopper (valve shut) before letting go.

Requires: PySide6, pyqtgraph, numpy, pyserial (see requirements.txt)
"""

import argparse
import csv
import html
import math
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QSize, Qt, QTimer
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (QApplication, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel,
                               QDoubleSpinBox, QListWidget, QListWidgetItem, QMainWindow, QMessageBox,
                               QPushButton, QScrollArea, QSlider, QSpinBox, QVBoxLayout, QWidget)

import link as linkmod
import style as S
from protocol import (ABORTS, CAL_CLEAR, CAL_LASER, CAL_NONE, CAL_SPAN, CAL_ZERO, CTRL_MANUAL,
                      CTRL_MODES, DEFAULT_PROFILE, FLAG_DUAL_I2C, FLAG_POWER_CAUSE_MASK,
                      FLAG_POWER_CAUSE_SHIFT, FLAG_POWER_LATCH, FLAG_POWER_TRIP, FLAG_SERVO_OFF,
                      GAIN_LIMITS, GAINS_DEFAULTS, POWER_CAUSES,
                      GAINS_RESULTS, GAINS_SET, HEALTH, HOP_MM, HOVER_MS, PHASE_IDLE, PHASES, REASONS,
                      STATES, V_DOWN_MMS, V_UP_MMS, Gains, board_mac)

SCRIPT_DIR = Path(__file__).resolve().parent
RECORDINGS_DIR = SCRIPT_DIR / "recordings"

WINDOW_S = 10.0       # plot history
UI_PERIOD_MS = 33     # ~30 Hz redraw
HISTORY_CAP = 8192    # samples kept per channel (41 s at 200 Hz); must hold WINDOW_S twice over
LOG_MAX = 300         # event log rows kept
LIVE_AGE_S = 0.5      # telemetry older than this = "No telemetry"
SIDE_WIDTH = 360
# Gains panel: (group, [(key, label, decimals, step), ...]). Two per row.
GAIN_GROUPS = [
    ("Both controllers", [("hover_pct", "Hover %", 1, 1.0)]),
    ("PD position", [("pd_kp", "Kp", 3, 1.0), ("pd_kd", "Kd", 3, 1.0)]),
    ("Cascade · position → v_z", [("pos_kp", "Kp", 3, 0.1), ("pos_ki", "Ki", 3, 0.1),
                                  ("pos_kd", "Kd", 3, 0.1), ("vz_max", "v max", 2, 0.05)]),
    ("Cascade · v_z → valve", [("vel_kp", "Kp", 3, 1.0), ("vel_ki", "Ki", 3, 1.0),
                               ("vel_kd", "Kd", 3, 0.5), ("vel_i_max", "I max", 1, 1.0)]),
]
TX_POWERS = [("Full power", 0), ("20 dBm", 20), ("15 dBm", 15), ("10 dBm", 10), ("5 dBm", 5), ("2 dBm", 2)]

RAD2DEG = 180 / math.pi
# Anything here can be picked in a plot. (name, unit, getter, decimals)
CHANNELS = [
    ("Altitude z", "m", lambda t: t.z, 3),
    ("Setpoint z", "m", lambda t: t.z_sp, 3),
    ("Setpoint v_z", "m/s", lambda t: t.vz_sp, 3),
    ("Velocity v_z", "m/s", lambda t: t.vz, 3),
    ("Accel bias", "m/s²", lambda t: t.accel_bias, 3),
    ("Laser z", "m", lambda t: t.laser_z, 3),
    ("Accel x", "m/s²", lambda t: t.accel[0], 2),
    ("Accel y", "m/s²", lambda t: t.accel[1], 2),
    ("Accel z", "m/s²", lambda t: t.accel[2], 2),
    ("Gyro x", "°/s", lambda t: t.gyro[0] * RAD2DEG, 1),
    ("Gyro y", "°/s", lambda t: t.gyro[1] * RAD2DEG, 1),
    ("Gyro z", "°/s", lambda t: t.gyro[2] * RAD2DEG, 1),
    ("Pressure", "psi", lambda t: t.psi, 1),
    ("PT loop", "mA", lambda t: t.pt_ma, 3),
    ("Valve temp", "°C", lambda t: t.temp_c, 2),
    ("Servo rail", "V", lambda t: t.servo_v, 2),
    ("Valve", "%", lambda t: t.valve_pct, 0),
    ("Signal", "dBm", lambda t: t.rssi or math.nan, 0),
    ("Cmd gap max", "ms", lambda t: t.max_gap_ms, 0),
    ("Round trip", "ms", lambda t: t.rtt_ms, 1),
    ("I2C errors", "", lambda t: t.i2c_errors, 0),
]
DEFAULT_PLOTS = ["Altitude z", "Setpoint z", "Pressure", "Accel z"]

CSV_HEAD = ["host_ms", "seq", "time_ms", "state", "reason", "valve_pct", "health", "flags",
            "z_m", "vz_ms", "accel_bias", "laser_z_m", "ax", "ay", "az", "gx_rads", "gy_rads", "gz_rads",
            "temp_c", "pt_ma", "psi", "servo_v", "failsafes", "cmd_seq", "rtt_ms", "max_gap_ms", "rssi",
            "tx_power", "laser_offset_m", "pt_zero_ma", "pt_span", "cal_ack", "i2c_errors",
            "ctrl_mode", "phase", "abort_reason", "z_sp_m", "vz_sp_ms", "hop_mm", "v_up_mms",
            "v_down_mms", "hover_ms"]


def fmt(x, d):
    return f"{x:.{d}f}" if isinstance(x, (int, float)) and math.isfinite(x) else "—"


def state_name(s):
    return STATES[s] if s < len(STATES) else str(s)


def reason_name(r):
    return REASONS[r] if r < len(REASONS) else str(r)


def phase_name(p):
    return PHASES[p] if p < len(PHASES) else str(p)


def abort_name(a):
    return ABORTS[a] if a < len(ABORTS) else str(a)


def ctrl_name(m):
    return CTRL_MODES[m] if m < len(CTRL_MODES) else str(m)


class History:
    """Recent telemetry as numpy arrays: one time column and a column per CHANNELS entry.

    Telemetry comes at 200 Hz, so a plot shows ~2000 points; keeping them in arrays means
    a redraw is a slice, not a conversion of 2000 Python tuples."""

    def __init__(self, channels, cap=HISTORY_CAP):
        self.t = np.empty(cap)
        self.v = np.empty((cap, channels))
        self.n = 0

    def clear(self):
        self.n = 0

    def append(self, t_ms, values):
        if self.n == len(self.t):  # full: keep the newer half (amortised, no per-sample shift)
            keep = len(self.t) // 2
            self.t[:keep] = self.t[-keep:]
            self.v[:keep] = self.v[-keep:]
            self.n = keep
        self.t[self.n] = t_ms
        self.v[self.n] = values
        self.n += 1

    def window(self, k, t_end_ms):
        """(times, values) of channel k over the last WINDOW_S, times in ms."""
        t = self.t[:self.n]
        start = np.searchsorted(t, t_end_ms - WINDOW_S * 1000)
        return t[start:], self.v[start:self.n, k]


def set_text(widget, text):
    """setText only on a change: re-laying out a label 30 times a second adds up."""
    if widget.text() != text:
        widget.setText(text)


def set_style(widget, css):
    """setStyleSheet only on a change: each call re-polishes the widget."""
    if widget.styleSheet() != css:
        widget.setStyleSheet(css)


def csv_row(t):
    vals = [int(time.time() * 1000), t.seq, t.time_ms, t.state, t.reason, t.valve_pct, t.health, t.flags,
            t.z, t.vz, t.accel_bias, t.laser_z, *t.accel, *t.gyro, t.temp_c, t.pt_ma, t.psi, t.servo_v,
            t.failsafes, t.cmd_seq, t.rtt_ms, t.max_gap_ms, t.rssi, t.tx_power, t.laser_offset_m,
            t.pt_zero_ma, t.pt_span, t.cal_ack, t.i2c_errors,
            t.ctrl_mode, t.phase, t.abort_reason, t.z_sp, t.vz_sp, t.hop_mm, t.v_up_mms,
            t.v_down_mms, t.hover_ms]
    return ["" if isinstance(v, float) and math.isnan(v) else v for v in vals]


# ---- small widget helpers ----
def label(text="", role=None, size=11, weight=QFont.Normal, mono=True, spacing=0.0, upper=False):
    w = QLabel(text.upper() if upper else text)
    w.setFont(S.font(size, weight, mono, spacing))
    if role:
        w.setProperty("role", role)
    return w


def caps(text, role="dim"):
    """The .label class: mono 10px, uppercase, dim."""
    return label(text, role, upper=True, **S.LABEL)


def button(text, variant=None, big=False):
    b = QPushButton(text.upper())
    b.setCursor(Qt.PointingHandCursor)
    if variant:
        b.setProperty("variant", variant)
    if big:
        b.setProperty("size", "big")
    return b


def hint(text):
    h = label(text, "hint", size=11, mono=False)
    h.setWordWrap(True)
    return h


class ValueAxis(pg.AxisItem):
    """Y axis labelled like index.html: two decimals, no "-0.00"."""

    def tickStrings(self, values, scale, spacing):
        return [f"{v * scale:.2f}".replace("-0.00", "0.00") for v in values]


class Plot(QFrame):
    """One plot cell: channel picker, big current value, 10 s trace."""

    def __init__(self, default, right, bottom):
        super().__init__(objectName="plotcell")
        self.setProperty("right", "1" if right else "0")
        self.setProperty("bottom", "1" if bottom else "0")
        self.channel = QComboBox(objectName="channel")
        self.channel.setFont(S.label_font())
        self.channel.setCursor(Qt.PointingHandCursor)
        for name, unit, _, _ in CHANNELS:
            self.channel.addItem((f"{name} ({unit})" if unit else name).upper())
        self.channel.setCurrentIndex([c[0] for c in CHANNELS].index(default))
        self.channel.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.value = QLabel()
        self.value.setTextFormat(Qt.RichText)
        self.value.setAlignment(Qt.AlignRight | Qt.AlignTop)

        self.plot = pg.PlotWidget(background=S.BG, axisItems={"left": ValueAxis("left")})
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.hideButtons()
        self.plot.setMenuEnabled(False)
        item = self.plot.getPlotItem()
        item.setContentsMargins(0, 0, 16, 0)  # room for the "0s" label
        item.getViewBox().setDefaultPadding(0.1)
        tick_font = S.font(10)
        left, bottom_ax = item.getAxis("left"), item.getAxis("bottom")
        for ax in (left, bottom_ax):
            ax.setStyle(tickLength=0, tickTextOffset=8, tickFont=tick_font, hideOverlappingLabels=False)
            ax.setTextPen(pg.mkPen(S.FAINT))
        left.setPen(pg.mkPen(S.GRID))
        left.setGrid(255)          # grid lines in the GRID colour
        left.setWidth(56)
        bottom_ax.setPen(pg.mkPen(S.INK))
        bottom_ax.setTicks([[(-s, f"-{s}s" if s else "0s") for s in range(10, -1, -2)], []])
        # Vertical grid as fixed lines, not bottom_ax.setGrid(): with a grid on, pyqtgraph
        # clips the axis labels to the plot and drops the "-10s" and "0s" at the edges
        for s in range(0, 11, 2):
            self.plot.addItem(pg.InfiniteLine(-s, angle=90, pen=pg.mkPen(S.GRID)), ignoreBounds=True)
        self.plot.setXRange(-WINDOW_S, 0, padding=0)
        # Width 1, not 1.25: a 1 px pen takes Qt's hairline path, ~10x faster to antialias.
        # Stroking every point of a noisy 200 Hz trace with a wider pen is what made the
        # window lag (and starved the link threads of the GIL while it drew).
        self.curve = self.plot.plot(pen=pg.mkPen(S.INK, width=1), connect="finite",
                                    autoDownsampleFactor=1.0)
        # ~2000 points across a few hundred pixels: draw each pixel's min and max instead
        # (same picture, spikes kept)
        self.curve.setDownsampling(auto=True, method="peak")
        self.curve.setClipToView(True)
        self._shown = None

        head = QHBoxLayout()
        head.setContentsMargins(16, 12, 16, 4)
        head.addWidget(self.channel, 0, Qt.AlignTop)
        head.addStretch()
        head.addWidget(self.value)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 6)
        lay.setSpacing(0)
        lay.addLayout(head)
        lay.addWidget(self.plot, 1)
        self.set_value("—", self.unit())

    def unit(self):
        return CHANNELS[self.channel.currentIndex()][1]

    def set_value(self, text, unit):
        if (text, unit) == self._shown:
            return
        self._shown = (text, unit)
        families = ", ".join(f"'{f}'" for f in S.MONO_LIGHT)
        unit_html = (f"<span style='font-size:13px; font-weight:400; color:{S.DIM}'>&nbsp;{html.escape(unit)}</span>"
                     if unit else "")
        self.value.setText(f"<span style=\"font-family:{families}; font-size:34px; font-weight:300\">"
                           f"{html.escape(text)}</span>{unit_html}")


class GroundStation(QMainWindow):
    def __init__(self, port_arg):
        super().__init__()
        self.setWindowTitle("Hopper Ground Station")
        self.resize(1440, 900)
        self.link = None
        self.latest = None
        self.last_rx = 0.0
        self.rx_times = deque()
        self.lost = 0
        self.last_seq = None
        self.cal_synced = False
        self.hopper_gains = None  # Gains the hopper last reported in use, None = no report yet
        self.gains_pending_shown = False
        self.history = History(len(CHANNELS))
        self.overflow = 0      # link.overflow already reported
        self.recording = None  # [file, csv writer, path, rows]
        self.go_sent = 0.0     # time.monotonic() of the last GO, to spot a refused one
        self.ground_mac = board_mac("GROUND_MAC")
        self.hopper_mac = board_mac("HOPPER_MAC")

        root = QWidget(objectName="root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self._build_header())
        body = QHBoxLayout()
        body.setSpacing(0)
        outer.addLayout(body, 1)

        plots = QWidget(objectName="plots")
        grid = QGridLayout(plots)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(0)
        self.plots = []
        for i, name in enumerate(DEFAULT_PLOTS):
            p = Plot(name, right=i % 2 == 0, bottom=i < 2)
            grid.addWidget(p, i // 2, i % 2)
            self.plots.append(p)
        body.addWidget(plots, 1)
        body.addWidget(self._build_side())

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.estop, context=Qt.ApplicationShortcut)
        self.timer = QTimer(self, timeout=self.tick)
        self.timer.start(UI_PERIOD_MS)

        self.refresh_ports(port_arg)
        self.set_connected(False)

    # ---- layout ----
    def _build_header(self):
        bar = QFrame(objectName="bar")
        bar.setFixedHeight(52)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        brand = QFrame(objectName="brand")
        bl = QHBoxLayout(brand)
        bl.setContentsMargins(20, 0, 20, 0)
        bl.setSpacing(10)
        bl.addWidget(label("→", size=14))
        bl.addWidget(label("HOPPER GS", size=12, weight=QFont.DemiBold, spacing=0.08))
        lay.addWidget(brand)

        self.stats = {}
        for key in ["State", "Valve", "Rx rate", "Last pkt", "Lost", "RTT", "Signal", "Hopper T+"]:
            cell = QFrame(objectName="cell")
            cell.setMinimumWidth(96)
            cl = QVBoxLayout(cell)
            cl.setContentsMargins(16, 0, 16, 0)
            cl.setSpacing(3)
            cl.addStretch()
            cl.addWidget(caps(key, role="label"))
            val = label("—", size=14)
            val.setFont(S.font(13 if key == "State" else 14, QFont.DemiBold if key == "State" else QFont.Normal,
                               spacing=0.06 if key == "State" else 0))
            cl.addWidget(val, 0, Qt.AlignLeft)
            cl.addStretch()
            lay.addWidget(cell)
            self.stats[key] = val
        lay.addStretch(1)

        conn = QFrame(objectName="conn")
        cl = QHBoxLayout(conn)
        cl.setContentsMargins(16, 0, 16, 0)
        cl.setSpacing(10)
        self.status = QLabel()
        self.status.setFont(S.label_font())
        cl.addWidget(self.status)
        self.port_box = QComboBox()
        self.port_box.setFixedWidth(250)
        cl.addWidget(self.port_box)
        refresh = button("↻")
        refresh.setToolTip("Look for boards again")
        refresh.clicked.connect(lambda: self.refresh_ports())
        cl.addWidget(refresh)
        self.record_btn = button("Record")
        self.record_btn.clicked.connect(self.toggle_recording)
        cl.addWidget(self.record_btn)
        self.connect_btn = button("Connect", "primary")
        self.connect_btn.clicked.connect(lambda: self.disconnect() if self.link else self.connect())
        cl.addWidget(self.connect_btn)
        lay.addWidget(conn)
        return bar

    def _section(self, title, extra=None):
        """A side panel section: grey header strip, then content with 16 px margins."""
        sec = QFrame(objectName="section")
        v = QVBoxLayout(sec)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(0)
        head = QLabel(objectName="sectionhead")
        head.setFont(S.label_font())
        head.setText(title.upper())
        if extra is not None:
            row = QHBoxLayout()
            row.setSpacing(0)
            row.addWidget(head, 1)
            extra.setObjectName("sectionhead")
            extra.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row.addWidget(extra)
            v.addLayout(row)
        else:
            v.addWidget(head)
        body = QVBoxLayout()
        body.setContentsMargins(16, 14, 16, 16)
        body.setSpacing(8)
        v.addLayout(body)
        return sec, body

    def _build_side(self):
        side = QWidget(objectName="side")
        lay = QVBoxLayout(side)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)

        sec, body = self._section("Command")
        row = QHBoxLayout()
        row.setSpacing(8)
        self.arm_btn = button("Arm", big=True)
        self.arm_btn.clicked.connect(self.toggle_arm)
        self.estop_btn = button("E-stop", "estop", big=True)
        self.estop_btn.clicked.connect(self.toggle_estop)
        row.addWidget(self.arm_btn)
        row.addWidget(self.estop_btn)
        body.addLayout(row)
        body.addSpacing(2)
        row = QHBoxLayout()
        row.setSpacing(8)
        k = caps("Valve")
        k.setFixedWidth(64)
        row.addWidget(k)
        self.valve_slider = QSlider(Qt.Horizontal, minimum=0, maximum=100)
        self.valve_slider.setCursor(Qt.PointingHandCursor)
        self.valve_slider.valueChanged.connect(self.set_valve)
        row.addWidget(self.valve_slider, 1)
        self.valve_label = label("0 %", size=12)
        self.valve_label.setFixedWidth(44)
        self.valve_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(self.valve_label)
        body.addLayout(row)
        row = QHBoxLayout()
        row.setSpacing(8)
        k = caps("Radio")
        k.setFixedWidth(64)
        row.addWidget(k)
        self.txpow = QComboBox()
        for name, dbm in TX_POWERS:
            self.txpow.addItem(name, dbm)
        self.txpow.currentIndexChanged.connect(self.set_tx_power)
        row.addWidget(self.txpow, 1)
        body.addLayout(row)
        lay.addWidget(sec)

        sec, body = self._section("Hop")
        row = QHBoxLayout()
        row.setSpacing(8)
        k = caps("Control")
        k.setFixedWidth(64)
        row.addWidget(k)
        self.ctrl_box = QComboBox()
        for i, name in enumerate(CTRL_MODES):
            self.ctrl_box.addItem(name, i)
        self.ctrl_box.currentIndexChanged.connect(self.set_ctrl_mode)
        row.addWidget(self.ctrl_box, 1)
        body.addLayout(row)
        self.profile = {}
        for key, name, unit, (lo, hi), step in [("z_sp_mm", "Height", "mm", HOP_MM, 50),
                                                ("v_up_mms", "Climb", "mm/s", V_UP_MMS, 25),
                                                ("v_down_mms", "Descent", "mm/s", V_DOWN_MMS, 25),
                                                ("hover_ms", "Hover", "ms", HOVER_MS, 500)]:
            row = QHBoxLayout()
            row.setSpacing(8)
            k = caps(name)
            k.setFixedWidth(64)
            row.addWidget(k)
            box = QSpinBox(minimum=lo, maximum=hi, singleStep=step, value=DEFAULT_PROFILE[key])
            box.lineEdit().setTextMargins(8, 0, 0, 0)
            box.valueChanged.connect(self.set_profile)
            row.addWidget(box, 1)
            u = caps(unit)
            u.setFixedWidth(40)
            row.addWidget(u)
            body.addLayout(row)
            self.profile[key] = box
        body.addSpacing(2)
        self.go_btn = button("Go", "primary", big=True)
        self.go_btn.clicked.connect(self.toggle_go)
        body.addWidget(self.go_btn)
        lay.addWidget(sec)

        self.gains_status = QLabel()
        self.gains_status.setFont(S.label_font())
        sec, body = self._section("Gains", self.gains_status)
        self.gain_boxes = {}
        for group, gains in GAIN_GROUPS:
            body.addWidget(caps(group, "hint"))
            grid = QGridLayout()
            grid.setHorizontalSpacing(8)
            grid.setVerticalSpacing(6)
            for i, (key, name, decimals, step) in enumerate(gains):
                k = caps(name)
                k.setFixedWidth(44)
                lo, hi = GAIN_LIMITS[key]
                box = QDoubleSpinBox(minimum=lo, maximum=hi, decimals=decimals, singleStep=step)
                box.lineEdit().setTextMargins(8, 0, 0, 0)
                box.setMinimumWidth(64)  # its own hint fits "1000.000" and overflows the panel
                box.setKeyboardTracking(False)
                box.valueChanged.connect(self.update_gains_controls)
                grid.addWidget(k, i // 2, (i % 2) * 2)
                grid.addWidget(box, i // 2, (i % 2) * 2 + 1)
                self.gain_boxes[key] = box
            grid.setColumnStretch(1, 1)
            grid.setColumnStretch(3, 1)
            body.addLayout(grid)
        body.addSpacing(2)
        row = QHBoxLayout()
        row.setSpacing(8)
        self.gains_apply = button("Apply", "primary")
        self.gains_apply.clicked.connect(lambda: self.send_gains(GAINS_SET))
        self.gains_revert = button("Revert")
        self.gains_revert.clicked.connect(self.revert_gains)
        self.gains_defaults = button("Defaults")
        self.gains_defaults.clicked.connect(lambda: self.send_gains(GAINS_DEFAULTS))
        for b in (self.gains_apply, self.gains_revert, self.gains_defaults):
            row.addWidget(b, 1)
        body.addLayout(row)
        body.addWidget(hint("Taken only while disarmed, and kept in the hopper's flash. "
                            "Orange = edited, not yet on the hopper."))
        lay.addWidget(sec)

        self.cal_status = QLabel()
        self.cal_status.setFont(S.label_font())
        sec, body = self._section("Calibration", self.cal_status)
        self.cal_zero = button("Zero pressure")
        self.cal_zero.clicked.connect(lambda: self.calibrate(CAL_ZERO, 0, "pressure zero"))
        self.cal_span = button("Pressure span")
        self.span_psi = QSpinBox(minimum=20, maximum=1000, value=100)
        self.cal_span.clicked.connect(
            lambda: self.calibrate(CAL_SPAN, self.span_psi.value(), f"pressure span at {self.span_psi.value()} psi"))
        self.cal_laser = button("Laser height")
        self.laser_mm = QSpinBox(minimum=0, maximum=2000, value=100)
        self.cal_laser.clicked.connect(
            lambda: self.calibrate(CAL_LASER, self.laser_mm.value(), f"laser height {self.laser_mm.value()} mm"))
        self.cal_clear = button("Clear to defaults")
        self.cal_clear.clicked.connect(lambda: self.calibrate(CAL_CLEAR, 0, "clear to defaults"))
        for btn, extra in [(self.cal_zero, hint("line vented")), (self.cal_span, (self.span_psi, "psi")),
                           (self.cal_laser, (self.laser_mm, "mm")), (self.cal_clear, None)]:
            row = QHBoxLayout()
            row.setSpacing(8)
            row.addWidget(btn, 1)
            if isinstance(extra, tuple):
                extra[0].setFixedWidth(72)
                extra[0].lineEdit().setTextMargins(8, 0, 0, 0)  # the QSS padding doesn't reach the editor
                row.addWidget(extra[0])
                u = caps(extra[1])
                u.setFixedWidth(24)
                row.addWidget(u)
            elif extra is not None:
                extra.setFixedWidth(104)
                row.addWidget(extra)
            body.addLayout(row)
        lay.addWidget(sec)

        sec, body = self._section("Health")
        chips = QGridLayout()
        chips.setSpacing(6)
        self.chips = []
        for i, (bit, name) in enumerate(HEALTH):
            chip = QLabel(textFormat=Qt.RichText)
            chip.setFont(S.font(10, QFont.Medium, spacing=0.06))
            chips.addWidget(chip, i // 3, i % 3)
            self.chips.append((bit, name, chip))
        self.i2c_chip = QLabel(textFormat=Qt.RichText)
        self.i2c_chip.setFont(S.font(10, QFont.Medium, spacing=0.06))
        chips.addWidget(self.i2c_chip, len(HEALTH) // 3, len(HEALTH) % 3)
        body.addLayout(chips)
        for _, name, chip in self.chips:
            self._chip(chip, name, None)
        self._chip(self.i2c_chip, "I2C", None)
        lay.addWidget(sec)

        sec, body = self._section("Event log")
        body.setContentsMargins(16, 6, 16, 16)
        self.log_list = QListWidget(objectName="log")
        self.log_list.setMinimumHeight(180)
        self.log_list.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        body.addWidget(self.log_list)
        sec.setStyleSheet("QFrame#section { border-bottom: none; }")
        lay.addWidget(sec, 1)

        scroll = QScrollArea(objectName="sidescroll")
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(SIDE_WIDTH)
        return scroll

    @staticmethod
    def _chip(chip, name, ok):
        """ok: True = green dot, black rule. False = red. None = no data (faint)."""
        dot, border, color = ((S.OK, S.INK, S.INK) if ok else (S.BAD, S.BAD, S.BAD) if ok is False
                              else (S.HAIR, S.HAIR, S.FAINT))
        set_text(chip, f"<span style='color:{dot}'>■</span>&nbsp;&nbsp;{html.escape(name.upper())}")
        set_style(chip, f"border: 1px solid {border}; color: {color}; padding: 4px 7px;")

    # ---- events ----
    def log(self, msg, error=False):
        row = QWidget()
        h = QHBoxLayout(row)
        h.setContentsMargins(0, 4, 0, 4)
        h.setSpacing(8)
        t = label(f"{datetime.now():%H:%M:%S}", size=11)
        t.setStyleSheet(f"color: {S.FAINT};")
        t.setFixedWidth(64)
        t.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        m = label(msg, size=11)
        m.setWordWrap(True)
        m.setStyleSheet(f"color: {S.BAD if error else S.INK};")
        h.addWidget(t)
        h.addWidget(m, 1)
        item = QListWidgetItem()
        width = SIDE_WIDTH - 32 - 12
        m.setFixedWidth(width - 72)
        item.setSizeHint(QSize(width, max(m.heightForWidth(width - 72), t.sizeHint().height()) + 9))
        self.log_list.insertItem(0, item)
        self.log_list.setItemWidget(item, row)
        while self.log_list.count() > LOG_MAX:  # thousands of row widgets slow the whole window
            self.log_list.takeItem(self.log_list.count() - 1)

    def refresh_ports(self, prefer=None):
        self.port_box.clear()
        ports = linkmod.find_ports()
        pick = 0
        for i, (device, _, serial_no) in enumerate(ports):
            name = device.rsplit("/", 1)[-1]
            if self.ground_mac and serial_no.upper().startswith(self.ground_mac):
                name, pick = "Ground · " + name, i
            elif self.hopper_mac and serial_no.upper().startswith(self.hopper_mac):
                name = "Hopper · " + name
            self.port_box.addItem(name, device)
            if prefer and device == prefer:
                pick = i
        if prefer and prefer not in [p[0] for p in ports]:
            self.port_box.insertItem(0, prefer.rsplit("/", 1)[-1], prefer)
            pick = 0
        if self.port_box.count() == 0:
            self.port_box.addItem("No boards found", None)
        self.port_box.setCurrentIndex(pick)

    def connect(self):
        device = self.port_box.currentData()
        if not device:
            self.log("No board selected: plug in the ground board and press ↻", True)
            return
        if self.port_box.currentText().startswith("Hopper"):
            self.log("That's the hopper's port: pick the ground board", True)
            return
        try:
            self.link = linkmod.Link(device)
        except Exception as e:  # pyserial raises several types for a busy or missing port
            self.log(f"Can't open {device}: {e}", True)
            return
        self.link.set_command(arm=False, estop=False, go=False, valve_pct=0, cal_op=CAL_NONE,
                              tx_power=self.txpow.currentData(), ctrl_mode=self.ctrl_box.currentData(),
                              **{k: b.value() for k, b in self.profile.items()})
        self.latest, self.last_seq, self.lost, self.cal_synced, self.overflow = None, None, 0, False, 0
        self.hopper_gains = None
        self.set_connected(True)
        self.log(f"Connected to the ground board on {device}")

    def disconnect(self, why=None):
        if self.link:
            self.link.close()
            self.link = None
        if self.recording:
            self.toggle_recording()
        self.set_connected(False)
        self.log(why or "Disconnected: hopper disarmed, valve shut")

    def set_connected(self, on):
        self.connect_btn.setText("DISCONNECT" if on else "CONNECT")
        S.set_variant(self.connect_btn, None if on else "primary")
        self.port_box.setEnabled(not on)
        for w in [self.arm_btn, self.estop_btn, self.valve_slider, self.txpow, self.cal_zero, self.span_psi,
                  self.cal_span, self.laser_mm, self.cal_laser, self.cal_clear, self.record_btn,
                  self.ctrl_box, *self.profile.values()]:
            w.setEnabled(on)
        self.update_controls()
        self.update_gains_controls()

    def update_controls(self):
        c = self.link.command() if self.link else None
        armed, estopped = bool(c and c.arm), bool(c and c.estop)
        self.arm_btn.setText("ARMED · DISARM" if armed else "ARM")
        S.set_variant(self.arm_btn, "armed" if armed else None)
        self.arm_btn.setEnabled(bool(self.link) and not estopped)
        self.estop_btn.setText("RELEASE E-STOP" if estopped else "E-STOP")
        S.set_variant(self.estop_btn, "estop-on" if estopped else "estop")
        valve = c.valve_pct if c else 0
        self.valve_slider.blockSignals(True)
        self.valve_slider.setValue(valve)
        self.valve_slider.blockSignals(False)
        self.valve_label.setText(f"{valve} %")
        self.cal_status.setText("SENDING…" if c and c.cal_op != CAL_NONE else "")
        going = bool(c and c.go)
        self.go_btn.setText("ABORT HOP" if going else "GO")
        S.set_variant(self.go_btn, "armed" if going else "primary")
        self.go_btn.setEnabled(bool(c and c.arm and not c.estop))

    def toggle_arm(self):
        arm = not self.link.command().arm
        self.link.set_command(arm=arm, **({} if arm else {"valve_pct": 0, "go": False}))
        self.update_controls()
        self.log("Arm requested" if arm else "Disarm")

    def estop(self):
        if not self.link or self.link.command().estop:
            return
        self.link.set_command(estop=True, arm=False, go=False, valve_pct=0)
        self.update_controls()
        self.log("E-STOP", True)

    def toggle_estop(self):
        if self.link.command().estop:
            self.link.set_command(estop=False)
            self.update_controls()
            self.log("E-stop released")
        else:
            self.estop()

    def set_valve(self, pct):
        if self.link:
            self.link.set_command(valve_pct=pct)
        self.valve_label.setText(f"{pct} %")

    def set_ctrl_mode(self):
        mode = self.ctrl_box.currentData()
        if self.link:
            self.link.set_command(ctrl_mode=mode)
            self.log(f"Controller: {ctrl_name(mode)}" + (" (taken at the next arm)" if self.link.command().arm else ""))

    def set_profile(self):
        if self.link:
            self.link.set_command(**{k: b.value() for k, b in self.profile.items()})

    def toggle_go(self):
        c = self.link.command()
        if c.go:
            self.link.set_command(go=False)
            self.log("Hop abort requested" if self.latest and self.latest.phase != PHASE_IDLE else "Go released")
        else:
            if c.ctrl_mode == CTRL_MANUAL:
                self.log("GO needs a closed-loop controller: pick one, then disarm and arm", True)
                return
            self.link.set_command(go=True)
            self.go_sent = time.monotonic()
            self.log("GO")
        self.update_controls()

    def set_tx_power(self):
        dbm = self.txpow.currentData()
        if self.link:
            self.link.set_command(tx_power=dbm)
            self.log("Hopper radio power " + (f"{dbm} dBm" if dbm else "full"))

    def calibrate(self, op, value, description):
        seq = self.link.command().cal_seq % 255 + 1  # never 0, the hopper's ack after boot
        self.link.set_command(cal_op=op, cal_seq=seq, cal_value=value)
        self.update_controls()
        self.log("Calibration: " + description)

    # ---- gains ----
    def form_gains(self) -> Gains:
        return Gains(**{k: b.value() for k, b in self.gain_boxes.items()})

    def edited_keys(self) -> list:
        """Gains in the form that differ from the hopper's (at the form's precision)."""
        if not self.hopper_gains:
            return []
        return [k for k, b in self.gain_boxes.items()
                if round(getattr(self.hopper_gains, k), b.decimals()) != round(b.value(), b.decimals())]

    def show_gains(self, g: Gains):
        for k, b in self.gain_boxes.items():
            b.blockSignals(True)
            b.setValue(getattr(g, k))
            b.blockSignals(False)
        self.update_gains_controls()

    def revert_gains(self):
        if self.hopper_gains:
            self.show_gains(self.hopper_gains)

    def send_gains(self, op):
        if self.link.command().arm or (self.latest and self.latest.state == 1):
            self.log("Gains are taken only while disarmed: disarm first", True)
            return
        g = self.form_gains()
        if not self.link.request_gains(op, g):
            self.log("The hopper hasn't reported its gains yet (old firmware?)", True)
            return
        if op == GAINS_DEFAULTS:
            self.log("Gains: back to defaults requested")
        else:
            self.log("Gains requested: " + ", ".join(f"{k} {getattr(g, k):g}" for k in self.edited_keys()))
        self.update_gains_controls()

    def on_gains_report(self, r, acked):
        first = self.hopper_gains is None
        keep_edits = not first and bool(self.edited_keys())
        self.hopper_gains = r.gains
        ok = acked and r.result == 0
        if first or ok or not keep_edits:
            self.show_gains(r.gains)  # otherwise keep edits not applied yet (or refused)
        else:
            self.update_gains_controls()
        if first:
            self.log("Hopper gains: " + ", ".join(f"{k} {v:.3g}" for k, v in vars(r.gains).items()))
        if acked:
            self.log("Gains " + (GAINS_RESULTS[r.result] if r.result < len(GAINS_RESULTS) else "refused"),
                     not ok)

    def update_gains_controls(self):
        have = bool(self.link) and self.hopper_gains is not None
        pending = bool(self.link) and self.link.gains_pending
        edited = set(self.edited_keys())
        for k, b in self.gain_boxes.items():
            b.setEnabled(have and not pending)
            S.set_variant(b, "edited" if k in edited else None)
        self.gains_apply.setEnabled(have and not pending and bool(edited))
        self.gains_revert.setEnabled(have and not pending and bool(edited))
        self.gains_defaults.setEnabled(have and not pending)
        self.gains_pending_shown = pending
        self.gains_status.setText("" if not self.link else "WAITING…" if not have else "SENDING…" if pending
                                  else f"{len(edited)} EDITED" if edited else "IN USE")

    def toggle_recording(self):
        if not self.recording:
            RECORDINGS_DIR.mkdir(exist_ok=True)
            path = RECORDINGS_DIR / f"hopper_{datetime.now():%Y-%m-%dT%H-%M-%S}.csv"
            f = open(path, "w", newline="")
            w = csv.writer(f)
            w.writerow(CSV_HEAD)
            self.recording = [f, w, path, 0]
            self.log(f"Recording to {path.name}")
        else:
            f, _, path, rows = self.recording
            f.close()
            self.recording = None
            self.log(f"Recording saved, {rows} rows: {path}")
        self.record_btn.setText("● STOP + SAVE" if self.recording else "RECORD")
        S.set_variant(self.record_btn, "rec-on" if self.recording else None)

    # ---- telemetry ----
    def on_packet(self, t):
        if self.last_seq is not None and t.seq < self.last_seq:
            self.log("Hopper restarted (seq went back)")
            self.history.clear()
        elif self.last_seq is not None and t.seq > self.last_seq + 1:
            self.lost += t.seq - self.last_seq - 1
        self.last_seq = t.seq
        if not self.cal_synced:
            # Continue from the hopper's last calibration number, so a new one is never
            # mistaken for one it already applied before this app started
            self.link.set_command(cal_seq=t.cal_ack)
            self.cal_synced = True
        if self.latest:
            self.on_changes(self.latest, t)
        self.latest = t
        self.last_rx = time.monotonic()
        self.rx_times.append(self.last_rx)
        self.history.append(t.time_ms, [get(t) for _, _, get, _ in CHANNELS])
        if self.recording:
            self.recording[1].writerow(csv_row(t))
            self.recording[3] += 1
        c = self.link.command()
        if c.cal_op != CAL_NONE and t.cal_ack == c.cal_seq:
            self.link.set_command(cal_op=CAL_NONE)
            self.update_controls()
            self.log(f"Calibration ack: zero {t.pt_zero_ma:.3f} mA, span {t.pt_span:.4f}, "
                     f"laser {t.laser_offset_m * 1000:.0f} mm")

    def on_changes(self, a, b):
        if a.state != b.state:
            why = f" ({reason_name(b.reason)})" if b.state == 2 else ""
            self.log(f"Hopper {state_name(b.state)}{why}", b.state == 2)
            if b.state == 2:
                self.link.set_command(arm=False, go=False, valve_pct=0)
                self.update_controls()
        if a.phase != b.phase:
            bad = b.phase == 5 or (b.phase == 4 and b.abort_reason)
            why = f" ({abort_name(b.abort_reason)})" if bad else ""
            self.log(f"Hop phase {phase_name(b.phase)}{why}", bad)
        c = self.link.command()
        if c.go and b.phase == PHASE_IDLE and time.monotonic() - self.go_sent > 0.5:
            # Still IDLE half a second after GO: the hopper refused it (abort_reason says why)
            self.log(f"GO refused: {abort_name(b.abort_reason)}", True)
            self.link.set_command(go=False)  # so the next GO is a fresh rising edge
            self.update_controls()
        for bit, name in HEALTH:
            if (a.health ^ b.health) & bit:
                self.log(f"{name} {'OK' if b.health & bit else 'LOST'}", not b.health & bit)
        if (a.flags ^ b.flags) & FLAG_DUAL_I2C:
            self.log("IMU moved to the shared I2C controller")
        if (a.flags ^ b.flags) & FLAG_POWER_TRIP:
            if b.flags & FLAG_POWER_TRIP:
                cause = (b.flags & FLAG_POWER_CAUSE_MASK) >> FLAG_POWER_CAUSE_SHIFT
                name = POWER_CAUSES[cause] if cause < len(POWER_CAUSES) else str(cause)
                self.log(f"POWER PROTECTION TRIPPED: {name}. Arming and GO blocked", True)
            else:
                self.log("Power protection cleared (servo supply steady)")
        if (a.flags ^ b.flags) & FLAG_SERVO_OFF:
            self.log("Servo signal OFF (power protection)" if b.flags & FLAG_SERVO_OFF
                     else "Servo signal back on", bool(b.flags & FLAG_SERVO_OFF))
        if b.flags & FLAG_POWER_LATCH and not a.flags & FLAG_POWER_LATCH:
            self.log("Power protection latched (3 trips): fix the battery, then restart the hopper", True)

    def tick(self):
        if self.link:
            while not self.link.events.empty():
                self.log(*self.link.events.get())
            packets = self.link.packets
            while packets:
                self.on_packet(packets.popleft())
            while self.link.gains_reports:
                self.on_gains_report(*self.link.gains_reports.popleft())
            if self.link.gains_pending != self.gains_pending_shown:  # e.g. the link gave up
                self.update_gains_controls()
            if self.link.overflow > self.overflow:
                self.log(f"Dashboard fell behind: skipped {self.link.overflow - self.overflow} packets", True)
                self.overflow = self.link.overflow
            if not self.link.alive:
                self.link = None
                self.disconnect("Link closed")
        self.render()

    def render(self):
        now = time.monotonic()
        while self.rx_times and self.rx_times[0] < now - 1:
            self.rx_times.popleft()
        t = self.latest
        age = now - self.last_rx if t else None
        live = t is not None and age < LIVE_AGE_S
        st = self.stats
        set_text(st["Rx rate"], f"{len(self.rx_times)} Hz" if t else "—")
        set_text(st["Last pkt"], ("—" if not t else f"{age * 1000:.0f} ms" if age < 1 else f"{age:.1f} s"))
        set_text(st["Lost"], str(self.lost) if t else "—")
        set_text(st["Hopper T+"], f"{t.time_ms / 1000:.2f} s" if t else "—")
        set_text(st["RTT"], f"{fmt(t.rtt_ms, 1)} ms" if t else "—")
        set_text(st["Signal"], f"{t.rssi} dBm" if t and t.rssi else "—")
        set_text(st["Valve"], f"{t.valve_pct} %" if t else "—")
        name = state_name(t.state) if t else "—"
        set_text(st["State"], name)
        bg, fg = S.STATE_BADGE.get(name, (None, S.BAR_TEXT)) if live else (None, S.BAR_TEXT)
        set_style(st["State"], f"background: {bg}; color: {fg}; padding: 1px 6px; margin-left: -6px;"
                  if bg else "padding: 1px 0;")

        # Square status dot: green pulsing = live, amber = connected without telemetry
        dot = (S.OK if (now % 1.2) < 0.6 else "#0a5c35") if live else S.WARN if self.link else S.BAR_RULE
        text = "Link live" if live else "No telemetry" if self.link else "Disconnected"
        set_text(self.status, f"<span style='color:{dot}'>■</span>&nbsp;&nbsp;"
                            f"<span style='color:{S.BAR_TEXT if live else S.BAR_DIM}'>{text.upper()}</span>")

        if t:
            for bit, name_, chip in self.chips:
                self._chip(chip, name_, bool(t.health & bit))
            dual = bool(t.flags & FLAG_DUAL_I2C)
            self._chip(self.i2c_chip, "I2C dual" if dual else "I2C shared", True if dual else None)

        t_end = t.time_ms if t else 0
        for p in self.plots:
            k = p.channel.currentIndex()
            _, unit, get, d = CHANNELS[k]
            p.set_value(fmt(get(t), d) if t else "—", unit)
            times, values = self.history.window(k, t_end)
            if len(times) > 1:
                p.curve.setData((times - t_end) / 1000, values)
            else:
                p.curve.setData([], [])

    def closeEvent(self, event):
        if self.link and self.link.command().arm:
            answer = QMessageBox.question(self, "Hopper is armed",
                                          "The hopper is ARMED. Close anyway? It will be disarmed (valve shut).")
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        if self.link:
            self.disconnect()
        event.accept()


def keep_awake():
    """Stop the OS throttling this app when its window is behind others or minimised.
    Both macOS (App Nap) and Windows 11 (power throttling) slow down a background app's
    timers, the link threads included: commands go out late and the hopper can FAILSAFE.
    Also keeps the laptop from idle-sleeping while the dashboard is open.
    Returns something to keep for the app's lifetime (or None). Best effort: never raises."""
    if sys.platform == "darwin":
        return _keep_awake_macos()
    if sys.platform == "win32":
        return _keep_awake_windows()
    return None  # Linux doesn't throttle background apps like this


def _keep_awake_windows():
    import ctypes
    from ctypes import wintypes

    class PowerThrottlingState(ctypes.Structure):  # PROCESS_POWER_THROTTLING_STATE
        _fields_ = [("Version", wintypes.ULONG), ("ControlMask", wintypes.ULONG),
                    ("StateMask", wintypes.ULONG)]

    try:
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        kernel32.SetProcessInformation.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p,
                                                   wintypes.DWORD]
        kernel32.SetProcessInformation.restype = wintypes.BOOL
        kernel32.SetThreadExecutionState.argtypes = [wintypes.DWORD]
        kernel32.SetThreadExecutionState.restype = wintypes.DWORD
        # ProcessPowerThrottling (4): control EXECUTION_SPEED (1) and IGNORE_TIMER_RESOLUTION
        # (4), state 0 = both off. Windows 10 before 1709 doesn't know it: returns False.
        state = PowerThrottlingState(1, 0x1 | 0x4, 0)
        kernel32.SetProcessInformation(kernel32.GetCurrentProcess(), 4, ctypes.byref(state),
                                       ctypes.sizeof(state))
        # 1 ms timer resolution for this process's sleeps and waits
        ctypes.WinDLL("winmm").timeBeginPeriod(1)
        # ES_CONTINUOUS | ES_SYSTEM_REQUIRED: no idle sleep (this thread, until it exits)
        kernel32.SetThreadExecutionState(0x80000000 | 0x00000001)
        return True
    except (OSError, AttributeError):
        return None


def _keep_awake_macos():
    import ctypes
    import ctypes.util
    try:
        ctypes.cdll.LoadLibrary("/System/Library/Frameworks/Foundation.framework/Foundation")
        objc = ctypes.cdll.LoadLibrary(ctypes.util.find_library("objc"))
        objc.objc_getClass.restype = objc.sel_registerName.restype = ctypes.c_void_p
        objc.objc_getClass.argtypes = objc.sel_registerName.argtypes = [ctypes.c_char_p]
        addr = ctypes.cast(objc.objc_msgSend, ctypes.c_void_p).value

        def send(obj, sel, *args, types=()):
            fn = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, *types)(addr)
            return fn(obj, objc.sel_registerName(sel), *args)

        info = send(objc.objc_getClass(b"NSProcessInfo"), b"processInfo")
        reason = send(objc.objc_getClass(b"NSString"), b"stringWithUTF8String:", b"Hopper command link",
                      types=(ctypes.c_char_p,))
        # NSActivityUserInitiated | NSActivityLatencyCritical
        token = send(info, b"beginActivityWithOptions:reason:", 0x00FFFFFF | 0xFF00000000, reason,
                     types=(ctypes.c_uint64, ctypes.c_void_p))
        return send(token, b"retain") if token else None
    except (OSError, AttributeError, TypeError):
        return None


def make_app(argv):
    pg.setConfigOptions(antialias=True, background=S.BG, foreground=S.FAINT)
    app = QApplication(argv)
    app.setFont(S.font(13, mono=False))
    app.setStyleSheet(S.QSS)
    return app


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    ap.add_argument("--port", help="ground board serial port (default: found from GROUND_MAC)")
    args = ap.parse_args()
    app = make_app(sys.argv)
    app.awake = keep_awake()
    win = GroundStation(args.port)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
