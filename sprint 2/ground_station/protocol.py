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

STATES = ["SAFE", "ARMED", "FAILSAFE"]
REASONS = ["none", "link lost", "E-stop"]
CAL_NONE, CAL_ZERO, CAL_SPAN, CAL_LASER, CAL_CLEAR = range(5)
HEALTH = [(0x01, "LASER"), (0x02, "IMU"), (0x04, "TEMP"), (0x08, "PRESSURE"), (0x10, "SERVO PWR")]
FLAG_DUAL_I2C = 0x01

CMD_PERIOD_S = 0.020   # 50 commands/s
LINK_TIMEOUT_S = 0.150  # no command for this long = hopper FAILSAFE

# telemetry_packet_t: type, seq, time_ms, then telemetry_data_t (90 bytes)
TELEMETRY_FMT = struct.Struct("<BII" "4f" "3f" "3f" "4f" "4BH" "IIHbB" "3fB" "BH")
# cmd_t
CMD_FMT = struct.Struct("<BBBBBBhBII")
assert TELEMETRY_FMT.size == 99 and CMD_FMT.size == 17

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


def encode_cmd(c: Command, seq: int, time_us: int) -> bytes:
    return CMD_FMT.pack(MSG_CMD, int(c.estop), int(c.arm), c.valve_pct, c.cal_op, c.cal_seq,
                        c.cal_value, c.tx_power, seq & 0xFFFFFFFF, time_us & 0xFFFFFFFF)


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
            elif self.state == 1:
                self.state = 2 if b == SYNC1 else 0
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
