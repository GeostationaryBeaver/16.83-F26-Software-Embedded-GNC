"""
USB link to the ground board, in two threads of its own.

The sender thread sends a command every 20 ms; the reader thread reads telemetry as it
arrives. Neither depends on the window: a slow or minimised window can't delay commands
and trip a FAILSAFE.

They are separate so that a write can never stop the reading. USB CDC has flow control
both ways: if we stop reading, the ground board blocks writing telemetry, stops reading
our commands, and then our write blocks too. With one thread doing both, that deadlocks
for good: the ground board still hears the hopper (green LED on) but nothing reaches
the dashboard, and no commands reach the hopper.
"""

import math
import queue
import threading
import time
from collections import deque
from dataclasses import replace

import serial
from serial.tools import list_ports

from protocol import (CMD_PERIOD_S, GAINS_FMT, MSG_GAINS, MSG_TELEMETRY, TELEMETRY_FMT, Command,
                      Deframer, Gains, decode_gains, decode_telemetry, encode_cmd, encode_set_gains,
                      frame)

USB_VIDS = (0x2341, 0x303A)  # Arduino (Nano ESP32), Espressif
DISARM_ON_CLOSE = 10          # disarm commands sent when the link closes (200 ms)
READ_TIMEOUT_S = 0.05         # how long a read waits for bytes (bounds how fast close() is)
WRITE_TIMEOUT_S = 0.1         # a command that can't go out in this long is dropped
GAINS_RESEND_EVERY = 5        # a gains request goes out every 5th command (100 ms) until acked
GAINS_GIVE_UP_S = 3.0         # no ack in this long = the request is dropped
BACKLOG = 1000                # packets kept for the window (20 s at 50 Hz); older ones dropped


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
    """Opens the port and runs the send and receive threads until close().

    Telemetry comes out of .packets (a deque: popleft() until it's empty), events out of
    .events (a queue of (message, is_error)). If the window falls more than BACKLOG
    packets behind, the oldest are dropped and counted in .overflow.
    The window changes the command with set_command(); the sender sends the latest one.

    Gains: request_gains() sends a MSG_SET_GAINS, repeated until the hopper acks it. Every
    MSG_GAINS report comes out of .gains_reports as (GainsReport, acked), acked = True for the
    one answering our latest request."""

    def __init__(self, device: str):
        self.packets = deque(maxlen=BACKLOG)  # Telemetry; append/popleft are thread-safe
        self.events = queue.Queue()           # (message, is_error)
        self.overflow = 0
        self.cmd_seq = 0
        self._cmd = Command()
        self.gains_reports = deque(maxlen=50)
        self._hopper_gains_seq = None  # last request seq the hopper reported, None = no report yet
        self._gains_req = None         # (seq, op, Gains, sent at) awaiting the ack
        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._port = serial.Serial()
        self._port.port, self._port.baudrate = device, 115200
        self._port.timeout, self._port.write_timeout = READ_TIMEOUT_S, WRITE_TIMEOUT_S
        # DTR on: the Nano ESP32 only sends over USB while DTR is set. RTS off: RTS high
        # resets an ESP32's USB-Serial/JTAG port, or reboots the Nano into its bootloader.
        self._port.dtr, self._port.rts = True, False
        self._port.open()
        self._threads = [threading.Thread(target=self._guard, args=(f,), name=f"link-{n}", daemon=True)
                         for n, f in (("send", self._send_loop), ("read", self._read_loop))]
        for t in self._threads:
            t.start()

    def command(self) -> Command:
        with self._lock:
            return replace(self._cmd)

    def set_command(self, **changes) -> Command:
        with self._lock:
            self._cmd = replace(self._cmd, **changes)
            return replace(self._cmd)

    def request_gains(self, op: int, gains: Gains) -> bool:
        """Sends new gains (GAINS_SET) or GAINS_DEFAULTS. False if the hopper hasn't reported
        its gains yet: the request number has to follow on from the hopper's last one."""
        with self._lock:
            if self._hopper_gains_seq is None:
                return False
            last = self._gains_req[0] if self._gains_req else self._hopper_gains_seq
            self._gains_req = (last % 255 + 1, op, gains, time.monotonic())  # never 0
            return True

    @property
    def gains_pending(self) -> bool:
        with self._lock:
            return self._gains_req is not None

    @property
    def alive(self) -> bool:
        return all(t.is_alive() for t in self._threads)

    def close(self):
        """Disarms the hopper (valve shut, SAFE), stops the threads and closes the port."""
        self.set_command(arm=False, go=False, valve_pct=0)
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        try:
            self._port.close()
        except (serial.SerialException, OSError):
            pass

    def _guard(self, loop):
        """Runs one thread's loop. A serial error ends both threads (alive goes False)."""
        try:
            loop()
        except (serial.SerialException, OSError) as e:
            if not self._stop.is_set():
                self.events.put((f"Serial error: {e}. No commands: the hopper will FAILSAFE if armed", True))
            self._stop.set()

    def _send(self) -> bool:
        self.cmd_seq += 1
        try:
            self._port.write(frame(encode_cmd(self.command(), self.cmd_seq, now_us())))
            return True
        except serial.SerialTimeoutException:
            return False

    def _send_gains(self, n: int):
        with self._lock:
            req = self._gains_req
            if req and time.monotonic() - req[3] > GAINS_GIVE_UP_S:
                self._gains_req = None
                self.events.put(("Gains not acknowledged by the hopper: not applied", True))
                return
        if req and n % GAINS_RESEND_EVERY == 0:
            try:
                self._port.write(frame(encode_set_gains(req[0], req[1], req[2])))
            except serial.SerialTimeoutException:
                pass

    def _send_loop(self):
        next_send = time.perf_counter()
        stalled = False  # a write timed out: say so once, and once when it recovers
        n = 0
        while not self._stop.is_set():
            ok = self._send()
            self._send_gains(n)
            n += 1
            if ok == stalled:
                stalled = not ok
                self.events.put(("Ground board isn't taking commands (USB write timed out)" if stalled
                                 else "Ground board is taking commands again", stalled))
            next_send += CMD_PERIOD_S
            t = time.perf_counter()
            if next_send < t:  # fell behind (e.g. laptop asleep): don't burst
                next_send = t + CMD_PERIOD_S
            # time.sleep, not self._stop.wait: on Windows a wait has ~15.6 ms resolution, a
            # sleep ~1 ms (Python 3.11+). close() still returns within one period.
            time.sleep(max(0.0, next_send - t))
        for _ in range(DISARM_ON_CLOSE):
            self._send()
            time.sleep(CMD_PERIOD_S)

    def _read_loop(self):
        deframer = Deframer()
        while not self._stop.is_set():
            # Waits up to READ_TIMEOUT_S for the first byte, then takes all that's there
            data = self._port.read(max(1, self._port.in_waiting))
            for p in deframer.feed(data):
                if p[0] == MSG_GAINS and len(p) == GAINS_FMT.size:
                    self._on_gains(decode_gains(p))
                    continue
                if p[0] != MSG_TELEMETRY:
                    continue
                if len(p) != TELEMETRY_FMT.size:
                    self.events.put((f"Telemetry is {len(p)} bytes, expected {TELEMETRY_FMT.size}: "
                                     "protocol.h and protocol.py differ", True))
                    continue
                tlm = decode_telemetry(p)
                rtt = ((now_us() - tlm.cmd_time_us) & 0xFFFFFFFF) / 1000
                tlm.rtt_ms = rtt if tlm.cmd_time_us and rtt < 10000 else math.nan
                if len(self.packets) == BACKLOG:
                    self.overflow += 1
                self.packets.append(tlm)

    def _on_gains(self, r):
        with self._lock:
            self._hopper_gains_seq = r.seq
            acked = self._gains_req is not None and r.seq == self._gains_req[0]
            if acked:
                self._gains_req = None
        self.gains_reports.append((r, acked))
