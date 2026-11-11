# Firmware (Arduino)

Arduino IDE sketches for the two Arduino Nano ESP32 boards: `hopper/` (the vehicle) and `ground/` (the ground station, see [Ground station](#ground-station) below).

## Hopper

Arduino IDE sketch for the hopper's Arduino Nano ESP32. It talks to the ground ESP (`ground/` below) over ESP-NOW, using the same packets.

```
hopper/
├── hopper.ino          setup(): valve shut first, then starts each module; passes commands on
└── src/
    ├── protocol.h      Packet definitions + board MAC addresses
    ├── drivers/        Hardware access only, no control logic
    │   ├── pins.h      Every pin, I2C address and analog constant (board rev F)
    │   ├── i2c_bus.*   Laser on Wire, IMU on Wire1; falls back to one shared controller
    │   ├── imu.*       LSM6DS3 accel (m/s^2) + gyro (rad/s), 416 Hz, +-8 g, 1000 dps
    │   ├── laser.*     VL53L0X range (mm), non-blocking
    │   ├── temp_probe.* DS18B20 (deg C), non-blocking state machine
    │   ├── pressure.*  PT1 4-20 mA loop current
    │   ├── servo.*     Valve servo PWM (held shut from power-up) + servo rail voltage
    │   ├── led.*       Onboard RGB LED
    │   └── radio.*     ESP-NOW link to the ground ESP, signal strength, TX power
    ├── calibration/
    │   └── calibration.* Pressure zero/span and laser offset, set from the ground, kept in flash
    ├── control/        Control loop, core 1, 100 Hz
    │   ├── control.*   Commands, link timeout, altitude filter, telemetry. Calls the controller
    │   ├── controller.* Altitude/rate PD: the law for every closed-loop mode
    │   ├── gains.h     Gain sets. The only file to edit between test runs
    │   ├── safety.*    SAFE / ARMED / FAILSAFE state machine: the only code that moves the valve
    │   ├── sensors.*   Reads every sensor on its schedule, health, retries dropped sensors
    │   └── altitude_kf.* Kalman filter: z, v_z, accel bias from IMU + laser
    └── telemetry/      Telemetry to the ground, core 0
        └── telemetry.* Lock-free ring buffer + radio task
```

Hardware handling (pins, sensor setup, I2C fallback, valve limits, failsafe rules, calibration) follows the avionics vehicle firmware TG-AV-003 rev F v1.1. Their own packet format (`'T','G'`, PROTO 3) is not used: the hopper talks to our ground ESP and dashboard with the packets in `protocol.h`, and telemetry carries every reading theirs sends.

## Safety

- The valve servo is held SHUT from power-up (rev F has no arming plug: the servo is powered whenever the battery is in). It moves only while ARMED.
- SAFE -> ARMED when the ground asks, the link is up, there is no E-stop, and the ground has sent "disarmed" at least once since boot or the last FAILSAFE.
- FAILSAFE: no command for 150 ms, or E-stop. Valve driven shut, latched until the ground disarms. A reboot always comes up SAFE.
- The ground (the dashboard) sends a command 50 times a second. LED: BLUE waiting, GREEN safe, YELLOW armed, RED failsafe / link lost.
- GNC: the control law is in `src/control/controller.cpp` and its numbers in `src/control/gains.h`. `control.cpp` is plumbing — see [Where the control code lives](#where-the-control-code-lives).

## Guidance modes

The dashboard's **Mode** picker sets `cmd_t.mode`; `cmd_t.setpoint` means whatever that mode says it means. `MODE_MANUAL` is 0, so manual valve control behaves exactly as before.

| Mode | `setpoint` | What the hopper does |
| --- | --- | --- |
| `MANUAL` | — | Valve follows `valve_pct`. No feedback. |
| `HOVER` | metres above the pad | Ramps its altitude setpoint at `sp_rate` to the commanded height and holds it. Enter 2 ft on the dashboard and it converts to metres. |
| `VELOCITY` | m/s, up positive | Tracks a climb rate. `-0.1` descends at 0.1 m/s. |
| `LAND` | — | Descends at the gain set's `land_rate`, then latches the valve shut below `land_cutoff`. |

One law serves all of them: `demand = valve_hover + kp*(sp_z - z) + kd*(sp_vz - vz)`. Hover is that setpoint standing still, a rate command is the same setpoint moving. Because rate tracking keeps the altitude term, a descent that falls behind is corrected rather than drifting, and mode switches are bumpless — the setpoint re-latches to wherever the vehicle is.

Tuning, in `src/control/gains.h`: change `ACTIVE_GAINS` to the set you want and re-upload; that one line is the only edit. `GAINS_FLIGHT` holds the known-good gains, `GAINS_BENCH` is a half-authority set for a first power-on. **`ACTIVE_GAINS` is `GAINS_BENCH`** until the loop has flown once on a tether. The gains that flew are echoed in telemetry (`kp`, `kd`) and recorded in the CSV, so a run always says which set produced it.

`valve_hover` matters more than either gain. A PD has no integral term, so the vehicle settles with a constant offset of `(true hover opening - valve_hover) / kp` — at `kp = 18`, 5.6 cm per 1 % of feedforward error. A hover that sits consistently low wants a higher `valve_hover`, not a higher `kp`.

Closed-loop modes need a trustworthy altitude. The filter's `z` only stays true while the laser keeps correcting it; with no laser it coasts on the IMU and drifts without bound. If no laser update lands for `EST_LASER_TIMEOUT_MS` (300 ms) the loop is inhibited and the valve shut, flagged `CTRL_NO_EST` in telemetry. The dashboard's readout shows the mode, the setpoint, the error, the raw demand and these flags; **Setpoint z** and **Valve demand** are plottable next to `z`.

### Where the control code lives

Three files, with a deliberately narrow seam between them, so you can retune without reading the plumbing and rewrite the law without touching it:

| File | Holds | Edit it when |
| --- | --- | --- |
| `gains.h` | Numbers only: gain sets, `ACTIVE_GAINS`, `EST_LASER_TIMEOUT_MS` | Retuning between runs |
| `controller.cpp` | The control law. No I/O, no timing, no valve | Changing the control architecture |
| `control.cpp` | Plumbing: sensors, commands, filter, telemetry, task | Adding a sensor or telemetry field |

`control.cpp` holds no control law and `controller.cpp` touches no hardware. They meet in exactly two places, both marked `THE CONTROLLER SEAM` in `control.cpp`:

- `valve_demand()` — the hook `safety.cpp` calls each tick while ARMED. Forwards to `controller_update()` and returns the opening.
- `send_telemetry()` — copies `controller_last()` and `controller_gains()` into the packet.

**To retune:** edit `gains.h`, re-upload. Nothing else, and `kp`/`kd` in the recording will say which set flew.

**To change the control architecture** (PID, LQR, gain scheduling, a cascaded rate loop): rewrite `controller.cpp` behind the five functions in `controller.h` and leave every other file alone.

```cpp
void controller_init();                       // zero your state, from control_start()
void controller_idle(const altitude_kf_t *);  // not ARMED: re-latch so the next arm is bumpless
const controller_out_t *controller_update(    // one tick while ARMED -> fill controller_out_t
    const altitude_kf_t *kf, const cmd_t *cmd, bool est_ok, float dt);
const controller_out_t *controller_last();    // last output, for telemetry
const gain_set_t *controller_gains();         // the set compiled in, for telemetry
```

Three rules the seam depends on. `controller_update()` is called **exactly once per tick** and may integrate state, which is why telemetry reads `controller_last()` instead of calling it again. It must be fast and non-blocking — no `delay()`, no I2C. And it only ever *returns* an opening: `safety.cpp` stays the only code that moves the valve, so FAILSAFE and the E-stop override whatever the law asks for.

If your law needs an input the signature doesn't carry, add it to `sensors_t` and pass it through. The two statics at the seam, `s_est_ok` and `s_dt_s`, show the pattern for values the fixed `valve_demand_fn_t` callback type cannot carry.

Adding a mode means a `control_mode_t` entry in both copies of `protocol.h`, a branch in `controller_update()`, and an entry in `MODE_CHOICES` (`ground_station.py`) and the `mode` picker (`index.html`).

Each module is a `.h`/`.cpp` pair, so you only edit the files you own. To add one (e.g. an IMU driver), add `src/drivers/imu.h` and `src/drivers/imu.cpp`, then include it by relative path where it's used: `#include "src/drivers/imu.h"` from `hopper.ino`, or `#include "../drivers/imu.h"` from another folder in `src/`. The Arduino IDE compiles everything under `src/` automatically.

### Sending telemetry from the control loop

```cpp
#include "../telemetry/telemetry.h"

telemetry_data_t data = { ... };
telemetry_push(&data);  // non-blocking, never takes a lock
```

`telemetry_push` must only be called from one task. To change what is sent, edit `telemetry_data_t` in `src/protocol.h`. **`hopper/src/protocol.h` and `ground/src/protocol.h` must stay identical**, and the dashboard decode in `sprint 2/ground_station/index.html` must match.

### Setup

1. Arduino IDE: install **Arduino ESP32 Boards** (Boards Manager), select **Arduino Nano ESP32**. Install the **VL53L0X** library by Pololu (Library Manager).
2. On Apple Silicon Macs, install Rosetta. Several of the IDE's tools for this board (ctags, esptool, dfu-util) are Intel-only, so compiling and uploading fail with "bad CPU type" without it:
   ```
   softwareupdate --install-rosetta
   ```
3. Fill in `HOPPER_MAC` and `GROUND_MAC` in both copies of `src/protocol.h`. To read a board's MAC:
   ```
   python -m esptool --chip esp32s3 read-mac
   ```
4. Open `hopper/hopper.ino` and upload.

## Ground station

Arduino IDE sketch for the ground station's Arduino Nano ESP32 (TG-AV-003 rev F ground board). It is a bridge between the hopper (ESP-NOW) and the dashboard on the laptop (USB, `sprint 2/ground_station/index.html`): packets go through unchanged in both directions. The dashboard sends the commands (50 per second) and shows the telemetry.

```
ground/
├── ground.ino          setup(): starts each module; loop(): the bridge
└── src/
    ├── protocol.h      Copy of hopper/src/protocol.h: keep the two identical
    ├── drivers/        Hardware access only
    │   ├── pins.h      LED and E-stop pins
    │   ├── lights.*    Green LED (D6), red LED (D5)
    │   ├── estop.*     E-stop on D2, normally closed to GND
    │   └── radio.*     Same ESP-NOW driver as the hopper's
    ├── usb/
    │   └── usb_link.*  Framed packets to and from the laptop
    └── bridge/
        └── bridge.*    Hopper <-> laptop relay, E-stop override, lights
```

Over USB each packet is framed as `0xAA 0x55 <len> <packet> <checksum = sum of packet bytes mod 256>`. Nothing else is printed, so don't use the Serial Monitor on this board (close it before connecting the dashboard).

- E-stop: while the switch on D2 is open (or its wire is cut), every command goes to the hopper with `estop = 1`, `arm = 0`, `valve_pct = 0`, whatever the dashboard sent. The hopper goes FAILSAFE and has to be disarmed and re-armed from the dashboard after the switch closes.
- Lights: GREEN = hearing the hopper. RED solid = hopper ARMED, RED blinking = hopper FAILSAFE. Both on for 0.5 s at power-up (lamp test).

Setup: same as the hopper (no extra libraries). Fill in `HOPPER_MAC` in `ground/src/protocol.h`, open `ground/ground.ino` and upload. Then open the dashboard in Chrome or Edge and press Connect.
