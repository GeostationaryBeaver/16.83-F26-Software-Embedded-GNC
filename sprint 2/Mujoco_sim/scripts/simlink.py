#!/usr/bin/env python3
"""Newline-delimited JSON over a socket, shared by gui.py and sim_worker.py.

One place for the framing so the two sides cannot disagree about it.

WHY A SOCKET AT ALL. MuJoCo's GLFW viewer and a tkinter control panel
cannot live in the same process: each is clean alone, but together they
corrupt the heap. Measured - launch_passive alone ran 269k steps and exited
cleanly, tkinter alone was clean, and the two together ran 165k steps and
then died with

    malloc(): unsorted double linked list corrupted

on exit. Two native event loops, one heap. So the viewer gets its own
process and this module is how the control panel talks to it.

PROTOCOL. One JSON object per line, both directions.

    control -> worker    {"cmd": "target",     "value": 2.0}
                         {"cmd": "propellant", "value": 4.0}
                         {"cmd": "pause"}  {"cmd": "resume"}
                         {"cmd": "reset"}
                         {"cmd": "quit"}
    worker -> control    {"t":.., "z":.., "vz":.., "mass":..,
                          "prop":.., "thrust":.., "setpoint":..}
                         {"error": "..."}   on anything unexpected
"""
import json
import socket


def send(sock, obj):
    """Write one JSON object plus its newline. Returns False if the peer is gone."""
    try:
        sock.sendall((json.dumps(obj) + "\n").encode())
        return True
    except (BrokenPipeError, ConnectionResetError, OSError):
        return False


class LineReader:
    """Non-blocking reader that yields complete JSON objects.

    *** THE BUG THIS EXISTS TO PREVENT. ***  The obvious way to poll a
    non-blocking socket is

        try:    chunk = sock.recv(4096)
        except Exception: pass          # <-- WRONG

    and a bare except there swallows genuine failures along with the
    "nothing to read yet" case. In a probe of this design that silently ate
    every command: the worker kept flying to its old target while telemetry
    reported z=2.0 against a commanded 1.0, with nothing logged anywhere.

    So: BlockingIOError means "no data yet" and is the ONLY thing treated as
    normal. Everything else is reported through errors() for the GUI to
    show. A half-received line is buffered, never parsed.
    """

    def __init__(self, sock):
        self._sock = sock
        self._buf = b""
        self._errors = []
        self.closed = False

    def errors(self):
        out, self._errors = self._errors, []
        return out

    def poll(self):
        """Return every complete object available right now, possibly empty."""
        if self.closed:
            return []
        try:
            chunk = self._sock.recv(65536)
        except BlockingIOError:
            chunk = b""                    # the one benign case
        except (ConnectionResetError, OSError) as e:
            self.closed = True
            self._errors.append(f"link lost: {type(e).__name__}: {e}")
            return []
        else:
            if chunk == b"":               # orderly shutdown by the peer
                self.closed = True
                return []

        self._buf += chunk
        out = []
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError as e:
                # Reported, not discarded - see the class docstring.
                self._errors.append(f"bad JSON {line[:60]!r}: {e}")
        return out


def listening_socket(host="127.0.0.1"):
    """A listening socket on an OS-chosen free port. Returns (sock, port)."""
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, 0))
    srv.listen(1)
    return srv, srv.getsockname()[1]
