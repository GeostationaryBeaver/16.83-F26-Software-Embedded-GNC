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
from PySide6.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QFrame, QGridLayout,
                               QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QMainWindow,
                               QMessageBox, QPushButton, QScrollArea, QSlider, QSpinBox,
                               QVBoxLayout, QWidget)

import link as linkmod
import style as S
from protocol import (CAL_CLEAR, CAL_LASER, CAL_NONE, CAL_SPAN, CAL_ZERO, FLAG_DUAL_I2C, HEALTH,
                      MODE_HOVER, MODE_LAND, MODE_MANUAL, MODE_VELOCITY, MODES, REASONS, SP_VZ_MAX,
                      SP_Z_MAX, STATES, board_mac, ctrl_flag_names, mode_name)

SCRIPT_DIR = Path(__file__).resolve().parent
RECORDINGS_DIR = SCRIPT_DIR / "recordings"

WINDOW_S = 10.0       # plot history
UI_PERIOD_MS = 33     # ~30 Hz redraw
LIVE_AGE_S = 0.5      # telemetry older than this = "No telemetry"
SIDE_WIDTH = 360
TX_POWERS = [("Full power", 0), ("20 dBm", 20), ("15 dBm", 15), ("10 dBm", 10), ("5 dBm", 5), ("2 dBm", 2)]
FT_PER_M = 3.280839895

# The Mode picker. Each mode reads Command.setpoint differently; the hint says how.
MODE_CHOICES = [
    (MODE_MANUAL, "Manual valve", "The valve follows the slider directly. No feedback."),
    (MODE_HOVER, "Hover at altitude",
     "The hopper ramps its setpoint to this height and holds it."),
    (MODE_VELOCITY, "Climb / descend at rate",
     "The hopper tracks this climb rate. Negative descends."),
    (MODE_LAND, "Land",
     "The hopper descends at the gain set's land_rate, then shuts the valve at land_cutoff."),
]

RAD2DEG = 180 / math.pi
# Anything here can be picked in a plot. (name, unit, getter, decimals)
CHANNELS = [
    ("Altitude z", "m", lambda t: t.z, 3),
    ("Setpoint z", "m", lambda t: t.sp_z, 3),
    ("Altitude error", "m", lambda t: t.sp_z - t.z, 3),
    ("Velocity v_z", "m/s", lambda t: t.vz, 3),
    ("Setpoint v_z", "m/s", lambda t: t.sp_vz, 3),
    ("Valve demand", "%", lambda t: t.demand, 1),
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
DEFAULT_PLOTS = ["Altitude z", "Setpoint z", "Valve demand", "Velocity v_z"]

CSV_HEAD = ["host_ms", "seq", "time_ms", "state", "reason", "valve_pct", "health", "flags",
            "z_m", "vz_ms", "accel_bias", "laser_z_m", "ax", "ay", "az", "gx_rads", "gy_rads", "gz_rads",
            "temp_c", "pt_ma", "psi", "servo_v", "failsafes", "cmd_seq", "rtt_ms", "max_gap_ms", "rssi",
            "tx_power", "laser_offset_m", "pt_zero_ma", "pt_span", "cal_ack", "i2c_errors",
            "mode", "ctrl_flags", "sp_z_m", "sp_vz_ms", "demand_pct", "kp", "kd"]


def fmt(x, d):
    return f"{x:.{d}f}" if isinstance(x, (int, float)) and math.isfinite(x) else "—"


def state_name(s):
    return STATES[s] if s < len(STATES) else str(s)


def reason_name(r):
    return REASONS[r] if r < len(REASONS) else str(r)


def csv_row(t):
    vals = [int(time.time() * 1000), t.seq, t.time_ms, t.state, t.reason, t.valve_pct, t.health, t.flags,
            t.z, t.vz, t.accel_bias, t.laser_z, *t.accel, *t.gyro, t.temp_c, t.pt_ma, t.psi, t.servo_v,
            t.failsafes, t.cmd_seq, t.rtt_ms, t.max_gap_ms, t.rssi, t.tx_power, t.laser_offset_m,
            t.pt_zero_ma, t.pt_span, t.cal_ack, t.i2c_errors,
            t.mode, t.ctrl_flags, t.sp_z, t.sp_vz, t.demand, t.kp, t.kd]
    return ["" if isinstance(v, float) and math.isnan(v) else v for v in vals]


# Every telemetry field, grouped. (label, getter) or (group title, None)
def readout_rows(win):
    return [
        ("Altitude filter", None),
        ("z", lambda t: fmt(t.z, 3) + " m"),
        ("v_z", lambda t: fmt(t.vz, 3) + " m/s"),
        ("accel bias", lambda t: fmt(t.accel_bias, 3) + " m/s²"),
        ("laser z", lambda t: fmt(t.laser_z, 3) + " m"),
        ("IMU", None),
        ("accel", lambda t: "  ".join(fmt(x, 2) for x in t.accel) + " m/s²"),
        ("gyro", lambda t: "  ".join(fmt(x * RAD2DEG, 1) for x in t.gyro) + " °/s"),
        ("Controller", None),
        ("mode", lambda t: mode_name(t.mode) + (" · " + ctrl_flag_names(t.ctrl_flags)
                                                if t.ctrl_flags else "")),
        ("setpoint z", lambda t: fmt(t.sp_z, 3) + " m"
            + (f"  ({fmt(t.sp_z * FT_PER_M, 2)} ft)" if math.isfinite(t.sp_z) else "")),
        ("setpoint v_z", lambda t: fmt(t.sp_vz, 3) + " m/s"),
        ("error", lambda t: fmt(t.sp_z - t.z, 3) + " m"),
        ("demand", lambda t: fmt(t.demand, 1) + " %"),
        ("gains", lambda t: f"kp {fmt(t.kp, 1)}  kd {fmt(t.kd, 1)}"),
        ("Valve + feed", None),
        ("state", lambda t: f"{state_name(t.state)} · last FAILSAFE {reason_name(t.reason)}"),
        ("valve", lambda t: f"{t.valve_pct} %"),
        ("pressure", lambda t: fmt(t.psi, 1) + " psi"),
        ("PT loop", lambda t: fmt(t.pt_ma, 3) + " mA"),
        ("inlet temp", lambda t: fmt(t.temp_c, 2) + " °C"),
        ("servo rail", lambda t: fmt(t.servo_v, 2) + " V"),
        ("failsafes", lambda t: str(t.failsafes)),
        ("Link", None),
        ("cmd seq", lambda t: f"{t.cmd_seq} (sent {win.link.cmd_seq if win.link else 0})"),
        ("round trip", lambda t: fmt(t.rtt_ms, 1) + " ms"),
        ("worst gap", lambda t: f"{t.max_gap_ms} ms / 2 s"),
        ("signal", lambda t: f"{t.rssi} dBm" if t.rssi else "—"),
        ("tx power", lambda t: f"{t.tx_power} dBm" if t.tx_power else "full"),
        ("Calibration", None),
        ("PT zero", lambda t: fmt(t.pt_zero_ma, 3) + " mA"),
        ("PT span", lambda t: fmt(t.pt_span, 4)),
        ("laser offset", lambda t: fmt(t.laser_offset_m * 1000, 0) + " mm"),
        ("cal ack", lambda t: f"{t.cal_ack} (sent {win.link.command().cal_seq if win.link else 0})"),
        ("Diagnostics", None),
        ("I2C", lambda t: "dual controllers" if t.flags & FLAG_DUAL_I2C else "shared controller"),
        ("I2C errors", lambda t: str(t.i2c_errors)),
    ]


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
        self.curve = self.plot.plot(pen=pg.mkPen(S.INK, width=1.25), connect="finite")

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
        self.series = [deque() for _ in CHANNELS]  # (time_ms, value)
        self.recording = None  # [file, csv writer, path, rows]
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

        # Mode, then one row per mode's own setpoint control. Only the active mode's row is
        # shown, so the number on screen is always the one being sent.
        row = QHBoxLayout()
        row.setSpacing(8)
        k = caps("Mode")
        k.setFixedWidth(64)
        row.addWidget(k)
        self.mode_box = QComboBox()
        for mode, name, _ in MODE_CHOICES:
            self.mode_box.addItem(name, mode)
        self.mode_box.currentIndexChanged.connect(self.set_mode)
        row.addWidget(self.mode_box, 1)
        body.addLayout(row)

        self.valve_row = QHBoxLayout()
        self.valve_row.setSpacing(8)
        k = caps("Valve")
        k.setFixedWidth(64)
        self.valve_row.addWidget(k)
        self.valve_slider = QSlider(Qt.Horizontal, minimum=0, maximum=100)
        self.valve_slider.setCursor(Qt.PointingHandCursor)
        self.valve_slider.valueChanged.connect(self.set_valve)
        self.valve_row.addWidget(self.valve_slider, 1)
        self.valve_label = label("0 %", size=12)
        self.valve_label.setFixedWidth(44)
        self.valve_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.valve_row.addWidget(self.valve_label)
        body.addLayout(self.valve_row)

        # Hover height is entered in feet, because the pad height is quoted in feet, and
        # converted here. SP_Z_MAX is the metric limit the hopper enforces.
        self.hover_row = QHBoxLayout()
        self.hover_row.setSpacing(8)
        k = caps("Hold")
        k.setFixedWidth(64)
        self.hover_row.addWidget(k)
        self.hover_ft = QDoubleSpinBox(minimum=0.0, maximum=SP_Z_MAX * FT_PER_M, value=2.0,
                                       singleStep=0.1, decimals=2)
        self.hover_ft.setFixedWidth(84)
        self.hover_ft.valueChanged.connect(self.set_hover)
        self.hover_row.addWidget(self.hover_ft)
        self.hover_row.addWidget(caps("ft"))
        self.hover_label = label("—", size=12)
        self.hover_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.hover_row.addWidget(self.hover_label, 1)
        body.addLayout(self.hover_row)

        self.rate_row = QHBoxLayout()
        self.rate_row.setSpacing(8)
        k = caps("Rate")
        k.setFixedWidth(64)
        self.rate_row.addWidget(k)
        # QSlider is integer-only: work in cm/s and scale to m/s on the way out.
        self.rate_slider = QSlider(Qt.Horizontal, minimum=int(-SP_VZ_MAX * 100),
                                   maximum=int(SP_VZ_MAX * 100), value=-10)
        self.rate_slider.setCursor(Qt.PointingHandCursor)
        self.rate_slider.valueChanged.connect(self.set_rate)
        self.rate_row.addWidget(self.rate_slider, 1)
        self.rate_label = label("-0.10 m/s", size=12)
        self.rate_label.setFixedWidth(68)
        self.rate_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.rate_row.addWidget(self.rate_label)
        body.addLayout(self.rate_row)

        self.mode_hint = hint("")
        body.addWidget(self.mode_hint)

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
        body.addSpacing(2)
        body.addWidget(hint("Commands go out 50 times a second from their own thread. None for 150 ms "
                            "(or E-stop) puts the hopper in FAILSAFE: valve shut until you disarm and "
                            "arm again. Esc = E-stop."))
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
        body.addSpacing(2)
        body.addWidget(hint("Saved in the hopper's flash. A refused calibration (no reading, transmitter "
                            "off) is acknowledged but changes nothing: the reason is on the hopper's "
                            "serial monitor."))
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

        sec, body = self._section("Telemetry")
        grid = QGridLayout()
        grid.setSpacing(0)
        grid.setColumnStretch(1, 1)
        self.readout = []
        for i, (name, get) in enumerate(readout_rows(self)):
            if get is None:
                g = caps(name, role="group")
                if i == 0:
                    g.setStyleSheet("padding-top: 0;")
                grid.addWidget(g, i, 0, 1, 2)
                continue
            k = label(name, "k", size=11)
            v = label("—", "v", size=11)
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(k, i, 0)
            grid.addWidget(v, i, 1)
            self.readout.append((v, get))
        body.addLayout(grid)
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
        chip.setText(f"<span style='color:{dot}'>■</span>&nbsp;&nbsp;{html.escape(name.upper())}")
        chip.setStyleSheet(f"border: 1px solid {border}; color: {color}; padding: 4px 7px;")

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
        self.link.set_command(arm=False, estop=False, valve_pct=0, cal_op=CAL_NONE,
                              mode=MODE_MANUAL, setpoint=0.0,
                              tx_power=self.txpow.currentData())
        self.mode_box.blockSignals(True)
        self.mode_box.setCurrentIndex(0)
        self.mode_box.blockSignals(False)
        self.latest, self.last_seq, self.lost, self.cal_synced = None, None, 0, False
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
        for w in [self.arm_btn, self.estop_btn, self.mode_box, self.valve_slider, self.hover_ft,
                  self.rate_slider, self.txpow, self.cal_zero, self.span_psi, self.cal_span,
                  self.laser_mm, self.cal_laser, self.cal_clear, self.record_btn]:
            w.setEnabled(on)
        self.update_controls()

    @staticmethod
    def _show_row(row, visible):
        for i in range(row.count()):
            w = row.itemAt(i).widget()
            if w:
                w.setVisible(visible)

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

        mode = self.mode()
        self.mode_box.blockSignals(True)
        self.mode_box.setCurrentIndex([m for m, _, _ in MODE_CHOICES].index(mode))
        self.mode_box.blockSignals(False)
        self._show_row(self.valve_row, mode == MODE_MANUAL)
        self._show_row(self.hover_row, mode == MODE_HOVER)
        self._show_row(self.rate_row, mode == MODE_VELOCITY)
        self.hover_label.setText(f"{self.hover_ft.value() / FT_PER_M:.3f} m")
        self.mode_hint.setText(next(h for m, _, h in MODE_CHOICES if m == mode))

        self.cal_status.setText("SENDING…" if c and c.cal_op != CAL_NONE else "")

    def toggle_arm(self):
        arm = not self.link.command().arm
        self.link.set_command(arm=arm, **({} if arm else {"valve_pct": 0}))
        self.update_controls()
        self.log(f"Arm requested ({mode_name(self.mode())})" if arm else "Disarm")

    def estop(self):
        if not self.link or self.link.command().estop:
            return
        self.link.set_command(estop=True, arm=False, valve_pct=0)
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

    def mode(self):
        return self.mode_box.currentData()

    def set_mode(self):
        """Mode changed. Each mode reads setpoint differently, so take it from that mode's own
        control rather than leaving the previous mode's number in place: switching hover ->
        rate would otherwise send a height as a climb rate."""
        mode = self.mode()
        if mode == MODE_HOVER:
            setpoint = self.hover_ft.value() / FT_PER_M
        elif mode == MODE_VELOCITY:
            setpoint = self.rate_slider.value() / 100.0
        else:
            setpoint = 0.0
        changes = {"mode": mode, "setpoint": setpoint}
        if mode != MODE_MANUAL:
            changes["valve_pct"] = 0  # the controller owns the valve now
        if self.link:
            self.link.set_command(**changes)
        self.update_controls()
        detail = (f" at {setpoint:.3f} m" if mode == MODE_HOVER else
                  f" at {setpoint:+.2f} m/s" if mode == MODE_VELOCITY else "")
        self.log(f"Mode {mode_name(mode)}{detail}")

    def set_hover(self, ft):
        m = ft / FT_PER_M
        self.hover_label.setText(f"{m:.3f} m")
        if self.link and self.mode() == MODE_HOVER:
            self.link.set_command(setpoint=m)

    def set_rate(self, cm_s):
        rate = cm_s / 100.0
        self.rate_label.setText(f"{rate:+.2f} m/s")
        if self.link and self.mode() == MODE_VELOCITY:
            self.link.set_command(setpoint=rate)

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
            for s in self.series:
                s.clear()
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
        for (_, _, get, _), s in zip(CHANNELS, self.series):
            s.append((t.time_ms, get(t)))
            while s and s[0][0] < t.time_ms - WINDOW_S * 1000:
                s.popleft()
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
                self.link.set_command(arm=False, valve_pct=0, mode=MODE_MANUAL, setpoint=0.0)
                self.mode_box.blockSignals(True)
                self.mode_box.setCurrentIndex(0)
                self.mode_box.blockSignals(False)
                self.update_controls()
        for bit, name in HEALTH:
            if (a.health ^ b.health) & bit:
                self.log(f"{name} {'OK' if b.health & bit else 'LOST'}", not b.health & bit)
        if (a.flags ^ b.flags) & FLAG_DUAL_I2C:
            self.log("IMU moved to the shared I2C controller")

    def tick(self):
        if self.link:
            while not self.link.events.empty():
                self.log(*self.link.events.get())
            while not self.link.packets.empty():
                self.on_packet(self.link.packets.get())
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
        st["Rx rate"].setText(f"{len(self.rx_times)} Hz" if t else "—")
        st["Last pkt"].setText(("—" if not t else f"{age * 1000:.0f} ms" if age < 1 else f"{age:.1f} s"))
        st["Lost"].setText(str(self.lost) if t else "—")
        st["Hopper T+"].setText(f"{t.time_ms / 1000:.2f} s" if t else "—")
        st["RTT"].setText(f"{fmt(t.rtt_ms, 1)} ms" if t else "—")
        st["Signal"].setText(f"{t.rssi} dBm" if t and t.rssi else "—")
        st["Valve"].setText(f"{t.valve_pct} %" if t else "—")
        name = state_name(t.state) if t else "—"
        st["State"].setText(name)
        bg, fg = S.STATE_BADGE.get(name, (None, S.BAR_TEXT)) if live else (None, S.BAR_TEXT)
        st["State"].setStyleSheet(f"background: {bg}; color: {fg}; padding: 1px 6px; margin-left: -6px;"
                                  if bg else "padding: 1px 0;")

        # Square status dot: green pulsing = live, amber = connected without telemetry
        dot = (S.OK if (now % 1.2) < 0.6 else "#0a5c35") if live else S.WARN if self.link else S.BAR_RULE
        text = "Link live" if live else "No telemetry" if self.link else "Disconnected"
        self.status.setText(f"<span style='color:{dot}'>■</span>&nbsp;&nbsp;"
                            f"<span style='color:{S.BAR_TEXT if live else S.BAR_DIM}'>{text.upper()}</span>")

        if t:
            for bit, name_, chip in self.chips:
                self._chip(chip, name_, bool(t.health & bit))
            dual = bool(t.flags & FLAG_DUAL_I2C)
            self._chip(self.i2c_chip, "I2C dual" if dual else "I2C shared", True if dual else None)
            for lbl, get in self.readout:
                lbl.setText(get(t))

        t_end = t.time_ms if t else 0
        for p in self.plots:
            k = p.channel.currentIndex()
            _, unit, get, d = CHANNELS[k]
            p.set_value(fmt(get(t), d) if t else "—", unit)
            s = self.series[k]
            if len(s) > 1:
                arr = np.array(s, dtype=float)
                p.curve.setData((arr[:, 0] - t_end) / 1000, arr[:, 1])
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
    win = GroundStation(args.port)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
