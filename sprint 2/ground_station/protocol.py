"""
Packets and USB framing for the hopper link.

MUST match firmware-arduino/hopper/src/protocol.h (and ground/src/protocol.h, its copy).
If you change a packet there, change TELEMETRY_FMT / CMD_FMT and the dataclasses here.

USB framing (firmware-arduino/ground/src/usb/usb_link.cpp):
    0xAA 0x55 <len> <packet: len bytes> <checksum = sum of packet bytes mod 256>
"""

import math
import re
import struct
from dataclasses import dataclass, field
from pathlib import Path

MSG_TELEMETRY = 1
MSG_CMD = 2
MSG_SET_GAINS = 3  # ground -> hopper: new controller gains
MSG_GAINS = 4      # hopper -> ground: gains in use, once a second and as the ack of each request

STATES = ["SAFE", "ARMED", "FAILSAFE"]
REASONS = ["none", "link lost", "E-stop", "power protection"]
PHASES = ["IDLE", "ASCENT", "HOVER", "DESCENT", "LANDED", "ABORT"]
PHASE_IDLE = 0
ABORTS = ["none", "go dropped", "sensor lost", "servo power lost", "timeout", "ceiling",
          "pre-flight check", "controller is MANUAL", "power protection"]
CTRL_MODES = ["manual", "PD position", "cascaded PID"]  # index = ctrl_mode_t
CTRL_MANUAL = 0

# Hop profile limits: the hopper clamps to these (hopper/src/control/flight.cpp)
HOP_MM = (100, 800)
V_UP_MMS = (50, 500)
V_DOWN_MMS = (50, 300)
HOVER_MS = (0, 10000)
DEFAULT_PROFILE = dict(z_sp_mm=500, v_up_mms=250, v_down_mms=200, hover_ms=3000)
CAL_NONE, CAL_ZERO, CAL_SPAN, CAL_LASER, CAL_CLEAR, CAL_PTRANGE = range(6)
HEALTH = [(0x01, "LASER"), (0x02, "IMU"), (0x04, "TEMP"), (0x08, "PRESSURE"), (0x10, "SERVO PWR")]
FLAG_DUAL_I2C = 0x01
# Power protection (hopper/src/control/power_guard.h)
FLAG_POWER_TRIP = 0x02   # tripped: arming and go refused
FLAG_SERVO_OFF = 0x04    # servo signal switched off
FLAG_POWER_LATCH = 0x08  # 3 trips since power-up: stays off until the hopper restarts
FLAG_POWER_CAUSE_SHIFT, FLAG_POWER_CAUSE_MASK = 4, 0x70
POWER_CAUSES = ["none", "servo rail low (battery flat / UBEC can't keep up)",
                "servo rail high (UBEC jumper not on 6.0 V)", "servo rail switching off and on (brown-out)",
                "pressure loop starved (12 V battery low)"]

CMD_PERIOD_S = 0.020   # 50 commands/s
LINK_TIMEOUT_S = 0.150  # no command for this long = hopper FAILSAFE

# telemetry_packet_t: type, seq, time_ms, then telemetry_data_t (109 bytes)
TELEMETRY_FMT = struct.Struct("<BII" "4f" "3f" "3f" "4f" "4BH" "IIHbB" "3fB" "BH" "BffBBHHHH")
# cmd_t
CMD_FMT = struct.Struct("<BBBBBBhBII" "BhBHHH")
# gains_packet_t: type, seq, op, result, then ctrl_gains_t (11 floats)
GAINS_FMT = struct.Struct("<BBBB" "11f")
assert TELEMETRY_FMT.size == 118 and CMD_FMT.size == 27 and GAINS_FMT.size == 48

GAINS_SET, GAINS_DEFAULTS = 1, 2  # gains_op_t
GAINS_RESULTS = ["applied", "refused: hopper is ARMED, disarm first", "refused: a gain is out of range"]

SYNC0, SYNC1 = 0xAA, 0x55
MAX_PACKET_LEN = 250

FIRMWARE_DIR = Path(__file__).resolve().parent.parent / "firmware-arduino"


@dataclass
class Telemetry:
    seq: int
    time_ms: int
    z: float
    vz: float
    accel_bias: float
    laser_z: float
    accel: tuple
    gyro: tuple
    temp_c: float
    pt_ma: float
    psi: float
    servo_v: float
    state: int
    reason: int
    valve_pct: int
    health: int
    failsafes: int
    cmd_seq: int
    cmd_time_us: int
    max_gap_ms: int
    rssi: int
    tx_power: int
    laser_offset_m: float
    pt_zero_ma: float
    pt_span: float
    cal_ack: int
    flags: int
    i2c_errors: int
    ctrl_mode: int
    z_sp: float        # flight phase setpoint (m), NaN outside a hop
    vz_sp: float
    phase: int
    abort_reason: int
    hop_mm: int        # profile in use (latched at go), the clamped request in IDLE
    v_up_mms: int
    v_down_mms: int
    hover_ms: int
    rtt_ms: float = math.nan  # filled in on receive, from cmd_time_us


def decode_telemetry(p: bytes) -> Telemetry:
    f = TELEMETRY_FMT.unpack(p)
    return Telemetry(
        seq=f[1], time_ms=f[2],
        z=f[3], vz=f[4], accel_bias=f[5], laser_z=f[6],
        accel=f[7:10], gyro=f[10:13],
        temp_c=f[13], pt_ma=f[14], psi=f[15], servo_v=f[16],
        state=f[17], reason=f[18], valve_pct=f[19], health=f[20], failsafes=f[21],
        cmd_seq=f[22], cmd_time_us=f[23], max_gap_ms=f[24], rssi=f[25], tx_power=f[26],
        laser_offset_m=f[27], pt_zero_ma=f[28], pt_span=f[29], cal_ack=f[30],
        flags=f[31], i2c_errors=f[32],
        ctrl_mode=f[33], z_sp=f[34], vz_sp=f[35], phase=f[36], abort_reason=f[37],
        hop_mm=f[38], v_up_mms=f[39], v_down_mms=f[40], hover_ms=f[41],
    )


@dataclass
class Command:
    estop: bool = False
    arm: bool = False
    valve_pct: int = 0
    cal_op: int = CAL_NONE
    cal_seq: int = 0
    cal_value: int = 0
    tx_power: int = 0  # dBm, 0 = full
    ctrl_mode: int = CTRL_MANUAL  # taken by the hopper only while not ARMED
    go: bool = False              # rising edge in IDLE starts a hop, dropping it aborts one
    z_sp_mm: int = DEFAULT_PROFILE["z_sp_mm"]       # hop height
    v_up_mms: int = DEFAULT_PROFILE["v_up_mms"]
    v_down_mms: int = DEFAULT_PROFILE["v_down_mms"]
    hover_ms: int = DEFAULT_PROFILE["hover_ms"]


def encode_cmd(c: Command, seq: int, time_us: int) -> bytes:
    return CMD_FMT.pack(MSG_CMD, int(c.estop), int(c.arm), c.valve_pct, c.cal_op, c.cal_seq,
                        c.cal_value, c.tx_power, seq & 0xFFFFFFFF, time_us & 0xFFFFFFFF,
                        c.ctrl_mode, c.z_sp_mm, int(c.go), c.v_up_mms, c.v_down_mms, c.hover_ms)


@dataclass
class Gains:
    """ctrl_gains_t, in the same order. Defaults in hopper/src/control/controllers.cpp."""
    hover_pct: float = 50.0
    pd_kp: float = 20.0
    pd_kd: float = 15.0
    pos_kp: float = 1.5
    pos_ki: float = 0.0
    pos_kd: float = 0.0
    vz_max: float = 0.6
    vel_kp: float = 25.0
    vel_ki: float = 10.0
    vel_kd: float = 2.0
    vel_i_max: float = 20.0

    def values(self) -> tuple:
        return tuple(getattr(self, k) for k in GAIN_KEYS)


GAIN_KEYS = list(Gains.__dataclass_fields__)
# What the hopper accepts (controller_set_gains in controllers.cpp): (lo, hi) per gain
GAIN_LIMITS = {k: (0.0, 1000.0) for k in GAIN_KEYS} | {
    "hover_pct": (0.0, 100.0), "vz_max": (0.01, 2.0), "vel_i_max": (0.0, 100.0)}


@dataclass
class GainsReport:
    seq: int      # last request the hopper handled, 0 = none since boot
    result: int   # GAINS_RESULTS index
    gains: Gains  # in use


def decode_gains(p: bytes) -> GainsReport:
    f = GAINS_FMT.unpack(p)
    return GainsReport(seq=f[1], result=f[3], gains=Gains(*f[4:]))


def encode_set_gains(seq: int, op: int, g: Gains) -> bytes:
    return GAINS_FMT.pack(MSG_SET_GAINS, seq, op, 0, *g.values())


def frame(packet: bytes) -> bytes:
    return bytes([SYNC0, SYNC1, len(packet)]) + packet + bytes([sum(packet) & 0xFF])


@dataclass
class Deframer:
    """Feed it raw USB bytes; it returns every complete packet with a good checksum."""
    state: int = 0
    length: int = 0
    buf: bytearray = field(default_factory=bytearray)

    def feed(self, data: bytes) -> list:
        packets = []
        for b in data:
            if self.state == 0:
                self.state = 1 if b == SYNC0 else 0
            elif self.state == 1:  # 0xAA 0xAA 0x55: the second 0xAA can still start a frame
                self.state = 2 if b == SYNC1 else 1 if b == SYNC0 else 0
            elif self.state == 2:
                self.length, self.buf = b, bytearray()
                self.state = 3 if 0 < b <= MAX_PACKET_LEN else 0
            elif self.state == 3:
                self.buf.append(b)
                if len(self.buf) == self.length:
                    self.state = 4
            else:
                self.state = 0
                if b == sum(self.buf) & 0xFF:
                    packets.append(bytes(self.buf))
        return packets


def board_mac(name: str) -> str | None:
    """HOPPER_MAC or GROUND_MAC from the firmware's protocol.h, as 'E8F60AD40968' (the USB
    serial number the board reports), or None if it isn't filled in."""
    try:
        text = (FIRMWARE_DIR / "ground" / "src" / "protocol.h").read_text()
    except OSError:
        return None
    m = re.search(rf"#define\s+{name}\s*\{{([^}}]*)\}}", text)
    if not m:
        return None
    octets = [int(x, 16) for x in m.group(1).split(",")]
    return None if not any(octets) else "".join(f"{o:02X}" for o in octets)
