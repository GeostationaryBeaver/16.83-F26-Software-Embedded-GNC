# Ground station

Laptop side of the hopper link. Plug the ground board (`firmware-arduino/ground`) into USB; it relays commands to the hopper and the hopper's telemetry back.

Two versions, same features and packets:

- **`ground_station.py`** (use this one): desktop app. Commands go out 50 times a second from their own thread, so a busy or minimised window can't slow them down and trip a FAILSAFE.
- `index.html`: the browser version (Chrome/Edge, Web Serial). Chrome slows timers in background tabs, so keep it in front.

## Run

```
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python ground_station.py
```

It picks the ground board's port from `GROUND_MAC` in `firmware-arduino/ground/src/protocol.h` (the board's USB serial number is its MAC), or pass `--port /dev/cu.usbmodem...`. Close the Arduino Serial Monitor and any dashboard tab first: only one program can have the port open.

- **Arm / E-stop / valve / radio power**: as in the browser version. `Esc` = E-stop.
- **Calibration**: zero, span, laser height, clear. Calibration numbers continue from the hopper's last one, so a new calibration is never mistaken for one it already applied.
- **Record**: writes every telemetry packet to `recordings/hopper_<time>.csv` as it arrives (same columns as the browser version).
- **Closing the window or Disconnect** sends 200 ms of disarm commands (valve shut, SAFE) before letting go of the port.

## Files

```
ground_station.py   The window: plots, controls, readouts, event log, recording
style.py            Colours, fonts and style sheet, matching index.html
link.py             Serial thread: 50 Hz commands, telemetry in
protocol.py         Packets and USB framing; must match firmware-arduino/hopper/src/protocol.h
```
