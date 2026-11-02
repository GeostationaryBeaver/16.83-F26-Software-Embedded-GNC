"""Log HX711 load-cell data from loadcell3.ino to a CSV.

Typical use (from WSL, Windows Python owns the COM port):

    python.exe LoadCellLogging.py --grams 214.44 --duration 100 --out trial1

The board is driven by serial commands ('t' tare, 'c <grams>' calibrate), which
loadcell3.ino accepts at any time from checkSerial() in its main loop. Nothing
here depends on catching the one-shot banner that setup() prints, so you do NOT
need to press reset first.
"""

import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports


# ---------------------------------------------------------------- CLI

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Record load-cell force to a CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    p.add_argument('--port', default='COM6',
                   help="serial port ('COM6' on Windows, '/dev/ttyACM0' on Linux)")
    p.add_argument('--baud', type=int, default=115200,
                   help='must match Serial.begin() in loadcell3.ino')
    p.add_argument('--duration', type=float, default=10.0, metavar='SEC',
                   help='length of the logged trial, in seconds')
    p.add_argument('--grams', type=float, default=None, metavar='G',
                   help='mass of the reference weight; omit to skip calibration')
    p.add_argument('--out', default=None, metavar='NAME',
                   help='output CSV name, .csv optional (default: timestamped)')
    p.add_argument('--force', action='store_true',
                   help='overwrite the output file if it already exists')
    p.add_argument('--skip-tare', action='store_true',
                   help='do not re-tare; keep the zero the board already has')
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
    if args.grams is not None and args.grams <= 0:
        p.error('--grams must be positive')
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


def await_reply(ser, success, what, deadline=12.0):
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
    say('Check that loadcell3.ino is the running sketch and the baud rate matches.')
    sys.exit(1)


# -------------------------------------------------------------- main

def resolve_output(args):
    name = args.out or time.strftime('load_cell_%Y%m%d_%H%M%S')
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

    # Opening the port resets the Arduino on most boards. Give setup() time to
    # finish and discard whatever it printed, then talk to it by command.
    time.sleep(2.0)
    ser.reset_input_buffer()

    if read_line(ser) is None:
        say('Port opened but the board is sending nothing on %s.' % args.port)
        say('Check the sketch is loaded and --baud matches Serial.begin().')
        sys.exit(1)

    # ---- tare: capture the zero with no test load on the cell
    if not args.skip_tare:
        pause('Remove any test load from the load cell, then press Enter.', args)
        send(ser, 't')
        await_reply(ser, '# Tare complete', 'Tare')

    # ---- calibrate: countsPerNewton stays 0 (force_N == nan) until we set it
    counts_per_newton = None
    if args.grams is None:
        say('\nNo --grams given: logging raw counts only, force_N will be nan.')
    else:
        pause('\nPlace %g g on the load cell, then press Enter.' % args.grams, args)
        send(ser, 'c %g' % args.grams)
        reply = await_reply(ser, '# Calibration complete', 'Calibration')
        try:
            counts_per_newton = reply.split(':', 1)[1].split()[0]
        except (IndexError, ValueError):
            counts_per_newton = 'unknown'
        pause('\nRemove the %g g mass, then press Enter to start logging.' % args.grams,
              args)

    # The board streams throughout the prompts, so drop the backlog of stale
    # pre-calibration rows and start the trial from a current sample.
    ser.reset_input_buffer()

    trial_ms = args.duration * 1000.0
    say('\nLogging %.1f s -> %s   (Ctrl-C to stop early)' % (args.duration, csv_path))

    rows = 0
    start_ms = None
    previous = None

    try:
        with open(csv_path, 'w') as f:
            f.write('# counts_per_newton=%s\n'
                    % (counts_per_newton if counts_per_newton else 'none'))
            f.write('# calibration_grams=%s\n'
                    % ('none' if args.grams is None else '%g' % args.grams))
            f.write('millis,raw,counts,force_N\n')

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
    except KeyboardInterrupt:
        say('\nInterrupted -- %d row(s) already written.' % rows)
    finally:
        ser.close()

    say('\nWrote %s  (%d rows)' % (csv_path, rows))
    if rows == 0:
        say('No rows captured. Is the HX711 wired to DT=3 / CLK=2?')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
