"""
USB link to the ground board, in its own thread.

The thread sends a command every 20 ms and reads telemetry in between. It doesn't depend
on the window: a slow or minimised window can't delay commands and trip a FAILSAFE.
"""

import math
import queue
import threading
import time
from dataclasses import replace

import serial
from serial.tools import list_ports

from protocol import (CMD_PERIOD_S, MSG_TELEMETRY, TELEMETRY_FMT, Command, Deframer,
                      decode_telemetry, encode_cmd, frame)

USB_VIDS = (0x2341, 0x303A)  # Arduino (Nano ESP32), Espressif
DISARM_ON_CLOSE = 10          # disarm commands sent when the link closes (200 ms)


def now_us() -> int:
    return (time.perf_counter_ns() // 1000) & 0xFFFFFFFF


def find_ports() -> list:
    """(device, description) for every Arduino / Espressif USB serial port."""
    out = []
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        if p.vid in USB_VIDS and p.device.startswith(("/dev/cu.", "COM", "/dev/ttyACM")):
            out.append((p.device, f"{p.device}  ({p.serial_number or p.description})", p.serial_number or ""))
    return out


class Link:
    """Opens the port and runs the send/receive thread until close().

    Telemetry and events come out of .packets and .events (thread-safe queues).
    The window changes the command with set_command(); the thread sends the latest one."""

    def __init__(self, device: str):
        self.packets = queue.Queue()   # Telemetry
        self.events = queue.Queue()    # (message, is_error)
        self.cmd_seq = 0
        self._cmd = Command()
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._port = serial.Serial()
        self._port.port, self._port.baudrate, self._port.timeout = device, 115200, 0
        # DTR on: the Nano ESP32 only sends over USB while DTR is set. RTS off: RTS high
        # resets an ESP32's USB-Serial/JTAG port, or reboots the Nano into its bootloader.
        self._port.dtr, self._port.rts = True, False
        self._port.open()
        self._thread = threading.Thread(target=self._run, name="link", daemon=True)
        self._thread.start()

    def command(self) -> Command:
        with self._lock:
            return replace(self._cmd)

    def set_command(self, **changes) -> Command:
        with self._lock:
            self._cmd = replace(self._cmd, **changes)
            return replace(self._cmd)

    @property
    def alive(self) -> bool:
        return self._thread.is_alive()

    def close(self):
        """Disarms the hopper (valve shut, SAFE), stops the thread and closes the port."""
        self.set_command(arm=False, valve_pct=0)
        self._stop.set()
        self._thread.join(timeout=2)

    def _send(self):
        self.cmd_seq += 1
        self._port.write(frame(encode_cmd(self.command(), self.cmd_seq, now_us())))

    def _run(self):
        deframer = Deframer()
        next_send = time.perf_counter()
        try:
            while not self._stop.is_set():
                t = time.perf_counter()
                if t >= next_send:
                    self._send()
                    next_send += CMD_PERIOD_S
                    if next_send < t:  # fell behind (e.g. laptop asleep): don't burst
                        next_send = t + CMD_PERIOD_S
                data = self._port.read(4096)
                for p in deframer.feed(data):
                    if p[0] == MSG_TELEMETRY:
                        if len(p) != TELEMETRY_FMT.size:
                            self.events.put((f"Telemetry is {len(p)} bytes, expected {TELEMETRY_FMT.size}: "
                                             "protocol.h and protocol.py differ", True))
                            continue
                        tlm = decode_telemetry(p)
                        rtt = ((now_us() - tlm.cmd_time_us) & 0xFFFFFFFF) / 1000
                        tlm.rtt_ms = rtt if tlm.cmd_time_us and rtt < 10000 else math.nan
                        self.packets.put(tlm)
                if not data:
                    time.sleep(min(0.002, max(0.0, next_send - time.perf_counter())))
            for _ in range(DISARM_ON_CLOSE):
                self._send()
                time.sleep(CMD_PERIOD_S)
        except (serial.SerialException, OSError) as e:
            self.events.put((f"Serial error: {e}. No commands: the hopper will FAILSAFE if armed", True))
        finally:
            try:
                self._port.close()
            except (serial.SerialException, OSError):
                pass
