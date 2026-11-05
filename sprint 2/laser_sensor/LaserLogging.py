"""Log VL53L0X range data from laser_logging.ino to a CSV.

Typical use (from WSL, Windows Python owns the COM port):

    python.exe LaserLogging.py --duration 60 --out hover1

The board is driven by serial commands ('z <mm>' calibrate, 'r' reset offset,
'p <0-3>' profile), which laser_logging.ino accepts at any time from
checkSerial() in its main loop. Nothing here depends on catching the one-shot
banner that setup() prints, so you do NOT need to press reset first.
"""

import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports


PROFILES = {
    0: 'default (33 ms, ~1.2 m)',
    1: 'high speed (20 ms, 50 Hz)',
    2: 'long range (33 ms, ~2 m)',
    3: 'high accuracy (200 ms, 5 Hz)',
}


# ---------------------------------------------------------------- CLI

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Record VL53L0X range to a CSV.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    p.add_argument('--port', default='COM6',
                   help="serial port ('COM6' on Windows, '/dev/ttyACM0' on Linux)")
    p.add_argument('--baud', type=int, default=115200,
                   help='must match Serial.begin() in laser_logging.ino')
    p.add_argument('--duration', type=float, default=10.0, metavar='SEC',
                   help='length of the logged trial, in seconds')
    p.add_argument('--zero', type=float, default=None, metavar='MM',
                   help='true target distance for offset calibration; omit to skip')
    p.add_argument('--profile', type=int, default=None, choices=sorted(PROFILES),
                   help='range profile to set before logging')
    p.add_argument('--out', default=None, metavar='NAME',
                   help='output CSV name, .csv optional (default: timestamped)')
    p.add_argument('--force', action='store_true',
                   help='overwrite the output file if it already exists')
    p.add_argument('--yes', '-y', action='store_true',
                   help='do not wait for Enter; pause --pause seconds instead')
    p.add_argument('--pause', type=float, default=5.0, metavar='SEC',
                   help='pause length when --yes is given')
    p.add_argument('--list-ports', action='store_true',
                   help='list detected serial ports and exit')
    p.add_argument('--quiet', '-q', action='store_true',
                   help='do not echo each data row while logging')

    args = p.parse_args(argv)
    if args.duration <= 0:
        p.error('--duration must be positive')
    if args.zero is not None and args.zero <= 0:
        p.error('--zero must be positive')
    return args


# ------------------------------------------------- terminal plumbing

# Under WSL -> python.exe interop neither stdin nor stdout is a tty: stdout is
# block-buffered, so prompts would sit unseen in the buffer. stderr is line
# buffered, so all interactive text goes there and is flushed explicitly.
def say(message=''):
    sys.stderr.write(message + '\n')
    sys.stderr.flush()


def pause(message, args):
    """Wait for the operator. Falls back to a timed pause without a usable stdin."""
    if args.yes:
        say('%s  (--yes: waiting %.1f s)' % (message, args.pause))
        time.sleep(args.pause)
        return

    sys.stderr.write(message + ' [Enter] ')
    sys.stderr.flush()
    line = sys.stdin.readline()
    if line == '':
        # EOF: stdin is not connected (piped, or redirected from /dev/null).
        say('\n(no stdin -- waiting %.1f s instead)' % args.pause)
        time.sleep(args.pause)
    else:
        say('')


# ------------------------------------------------------- serial layer

def open_port(args):
    """Open the serial port, or explain precisely why it could not be opened."""
    try:
        return serial.Serial(args.port, args.baud, timeout=2.0)
    except serial.SerialException as exc:
        text = str(exc)
        say('Could not open %s.\n  %s\n' % (args.port, text))

        try:
            found = list(list_ports.comports())
        except Exception:
            found = []

        if not any(p.device == args.port for p in found):
            say('%s is not present. Ports currently detected:' % args.port)
            for p in found:
                say('    %s  -  %s' % (p.device, p.description))
            if not found:
                say('    (none)')
            say('\nRe-run with --port set to whichever one is the Arduino.')
            say('The Nano ESP32 re-enumerates on every reset, so the port')
            say('number can change between runs.')
        elif 'not functioning' in text or 'Cannot configure' in text:
            # Windows error 31: the handle opens but the USB endpoint is wedged.
            say('The port exists but the USB device is in a bad state (error 31).')
            say('Unplug the USB cable, wait ~5 s, plug it back in, then retry.')
            say('Pressing the board reset button will NOT clear this.')
        else:
            say('Something else is holding the port. Close the Arduino IDE')
            say('Serial Monitor and any other terminal on %s.' % args.port)
        sys.exit(1)


def read_line(ser, attempts=15):
    """Read one decoded line. Returns None if the board stays silent."""
    for _ in range(attempts):
        raw = ser.readline()
        if raw:
            return raw.decode('utf-8', errors='replace').rstrip()
    return None


def send(ser, command):
    """Send a command, discarding anything queued so the reply is easy to spot."""
    ser.reset_input_buffer()
    ser.write((command + '\n').encode('utf-8'))
    ser.flush()


def await_reply(ser, success, what, deadline=20.0):
    """Read until a '# ' line starting with `success` arrives. Data rows are skipped."""
    limit = time.time() + deadline
    while time.time() < limit:
        line = read_line(ser)
        if line is None:
            continue
        if line.startswith('# ERROR'):
            say('%s failed: %s' % (what, line))
            sys.exit(1)
        if line.startswith(success):
            say(line)
            return line
        if line.startswith('#'):
            say(line)
    say('%s timed out after %.0f s -- no reply from the board.' % (what, deadline))
    say('Check that laser_logging.ino is the running sketch and the baud matches.')
    sys.exit(1)


# -------------------------------------------------------------- main

def resolve_output(args):
    name = args.out or time.strftime('laser_%Y%m%d_%H%M%S')
    if name.lower().endswith('.csv'):
        name = name[:-4]
    path = name + '.csv'
    if os.path.exists(path) and not args.force:
        say("'%s' already exists. Use --out NAME for a new file, or --force." % path)
        sys.exit(1)
    return path


def main(argv=None):
    args = parse_args(argv)

    if args.list_ports:
        for p in list_ports.comports():
            print('%s  -  %s' % (p.device, p.description))
        return 0

    csv_path = resolve_output(args)
    ser = open_port(args)

    # Opening the port resets the board on most variants. Give setup() time to
    # finish and discard whatever it printed, then talk to it by command.
    time.sleep(2.0)
    ser.reset_input_buffer()

    if read_line(ser) is None:
        say('Port opened but the board is sending nothing on %s.' % args.port)
        say('Check the sketch is loaded and --baud matches Serial.begin().')
        sys.exit(1)

    # ---- profile: set before calibrating, since it changes the timing budget
    profile = args.profile
    if profile is not None:
        send(ser, 'p %d' % profile)
        await_reply(ser, '# Profile', 'Profile change')

    # ---- offset calibration
    offset_mm = None
    if args.zero is None:
        say('\nNo --zero given: logging with whatever offset the board holds.')
        send(ser, '?')
        reply = await_reply(ser, '# offset_mm=', 'Settings query')
        try:
            offset_mm = reply.split('offset_mm=', 1)[1].split()[0]
        except (IndexError, ValueError):
            offset_mm = 'unknown'
    else:
        pause('\nPlace a flat target exactly %g mm away, then press Enter.'
              % args.zero, args)
        send(ser, 'z %g' % args.zero)
        reply = await_reply(ser, '# Calibration complete', 'Calibration')
        try:
            offset_mm = reply.split(':', 1)[1].split()[0]
        except (IndexError, ValueError):
            offset_mm = 'unknown'
        pause('\nPosition the sensor for the trial, then press Enter to log.',
              args)

    # The board streams throughout the prompts, so drop the backlog of stale
    # pre-calibration rows and start the trial from a current sample.
    ser.reset_input_buffer()

    trial_ms = args.duration * 1000.0
    say('\nLogging %.1f s -> %s   (Ctrl-C to stop early)'
        % (args.duration, csv_path))

    rows = 0
    valid_rows = 0
    start_ms = None
    previous = None

    try:
        with open(csv_path, 'w') as f:
            f.write('# offset_mm=%s\n' % (offset_mm if offset_mm else 'none'))
            f.write('# calibration_mm=%s\n'
                    % ('none' if args.zero is None else '%g' % args.zero))
            f.write('# profile=%s\n'
                    % ('unchanged' if profile is None else PROFILES[profile]))
            f.write('millis,raw_mm,range_mm,status\n')

            while True:
                line = read_line(ser)
                if line is None:
                    say('Board went quiet -- stopping early.')
                    break

                # '# ' lines are status or errors, never data.
                if line.startswith('#'):
                    say(line)
                    continue
                if line == previous:
                    continue

                fields = line.split(',')
                if len(fields) != 4:
                    continue
                try:
                    now_ms = int(fields[0])
                except ValueError:
                    continue

                if start_ms is None:
                    start_ms = now_ms
                if now_ms - start_ms > trial_ms:
                    break

                if not args.quiet:
                    say(line)
                previous = line
                f.write(line + '\n')
                rows += 1
                if fields[2] != 'nan':
                    valid_rows += 1
    except KeyboardInterrupt:
        say('\nInterrupted -- %d row(s) already written.' % rows)
    finally:
        ser.close()

    say('\nWrote %s  (%d rows, %d valid)' % (csv_path, rows, valid_rows))

    if rows == 0:
        say('No rows captured. Is the sensor wired to SDA=A4 / SCL=A5?')
        return 1

    # A trial that is mostly nan is usually a setup problem, not a sensor
    # fault: nothing in range, or an obstructed lens.
    if valid_rows == 0:
        say('Every row was nan. The sensor saw no valid target --')
        say('check the lens is unobstructed and aim at a wall 10-50 cm away.')
        return 1
    if valid_rows < rows * 0.5:
        say('Warning: %.0f%% of rows were nan. Check the target distance is'
            % (100.0 * (rows - valid_rows) / rows))
        say('inside 30-1600 mm and that nothing is in the 25 deg cone.')

    return 0


if __name__ == '__main__':
    sys.exit(main())
