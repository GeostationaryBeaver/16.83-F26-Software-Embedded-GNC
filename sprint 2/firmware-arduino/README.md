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
    │   ├── servo.*     Valve servo PWM (held shut from power-up, end points from flash), signal on/off, servo rail voltage
    │   ├── led.*       Onboard RGB LED
    │   └── radio.*     ESP-NOW link to the ground ESP, signal strength, TX power
    ├── calibration/
    │   └── calibration.* Pressure zero/span/range and laser offset, set from the ground, kept in flash
    ├── control/        Control loop, core 1, 250 Hz (telemetry sent at 50 Hz)
    │   ├── control.*   Commands, link timeout, altitude filter, flight phases + controller, telemetry
    │   ├── safety.*    SAFE / ARMED / FAILSAFE state machine: the only code that moves the valve
    │   ├── power_guard.* Power protection (avionics v1.3a): servo rail / pressure loop watch, servo signal cut
    │   ├── flight.*    Hop guidance: IDLE / ASCENT / HOVER / DESCENT / LANDED / ABORT, makes the setpoint
    │   ├── controllers.* + ctrl_pd.cpp, ctrl_cascade.cpp   Altitude control laws that track the setpoint
    │   ├── gains_store.*   Controller gains set from the ground (MSG_SET_GAINS), kept in flash
    │   ├── sensors.*   Reads every sensor on its schedule, health, retries dropped sensors
    │   └── altitude_kf.* Kalman filter: z, v_z, accel bias from IMU + laser
    └── telemetry/      Telemetry to the ground, core 0
        └── telemetry.* Lock-free ring buffer + radio task
test/                   Host tests, not part of the Arduino build (see flight_test.cpp for how to run)
```

Hardware handling (pins, sensor setup, I2C fallback, valve limits, failsafe rules, calibration) follows the avionics vehicle firmware TG-AV-003 rev F (v1.1, drivers updated to v1.3a: valve end points, pressure range, power protection). Their own packet format (`'T','G'`, PROTO 3) is not used: the hopper talks to our ground ESP and dashboard with the packets in `protocol.h`, and telemetry carries every reading theirs sends.

## Safety

- The valve servo is held SHUT from power-up (rev F has no arming plug: the servo is powered whenever the battery is in). It moves only while ARMED.
- SAFE -> ARMED when the ground asks, the link is up, there is no E-stop, and the ground has sent "disarmed" at least once since boot or the last FAILSAFE.
- FAILSAFE: no command for 150 ms, or E-stop. Valve driven shut, latched until the ground disarms. A reboot always comes up SAFE.
- The ground (the dashboard) sends a command 50 times a second. LED: BLUE waiting, GREEN safe, YELLOW armed, RED failsafe / link lost.
- **Valve end points**: shut 2500 us / open 1500 us by default (avionics bench, 2026-11-10, after re-indexing the servo on the valve; before that shut was 500 us). If the board's flash holds end points saved by the avionics firmware's `setshut` / `setopen` (`tg-servo` keys), those are used instead. This firmware has no console to set them: flash the avionics vehicle firmware, set them there, then flash this one back. **Check the valve really shuts at the shut end before pressurising.**
- **Pressure range**: 1000 psi transmitter by default. `CAL_PTRANGE` (cal_value = psi at 20 mA, e.g. 5000 for McMaster 3196K611) changes it and resets the span; then vent + zero, then span. Kept in flash with the same key as avionics (`ptFull`).
- **Power protection** (`control/power_guard.h`, from avionics v1.3a). Trips on: servo rail < 5.4 V for 0.5 s, > 6.9 V for 0.3 s, crossing 4.5 V 4 times in 10 s, or the pressure loop falling under 3.3 mA for 2 s after it was healthy. While tripped, arming and go are refused and telemetry shows no servo power. ARMED on the pad goes FAILSAFE (reason `power`); in flight it ABORTs into a controlled descent (in DESCENT it just keeps descending), so the servo can still land it. Once down with the valve shut for 0.6 s, the servo signal is switched off. Clears itself after 5 s of steady 5.6-6.6 V while the ground isn't asking to arm; after 3 trips since power-up it stays off until the hopper restarts. Telemetry `flags`: `FLAG_POWER_TRIP`, `FLAG_SERVO_OFF`, `FLAG_POWER_LATCH`, and the cause in bits 4-6. The dashboard logs each.

Neither firmware prints diagnostic text. Status is available through telemetry and LEDs;
the hopper does not wait for a USB serial connection during startup. Ground USB carries
only framed protocol packets.

## Hop (flight phases)

While ARMED, `src/control/flight.cpp` runs the hop: it decides where the hopper should be (the setpoint `z_sp` and its rate `vz_sp`), and the controller the ground picked (`ctrl_mode`: PD or cascade, latched while disarmed) decides the valve opening to get there.

- **IDLE**: valve follows the ground's manual slider. A `go` rising edge starts a hop if a closed-loop controller is picked, IMU / laser / pressure / servo power are healthy, and the hopper is on the ground and still. Otherwise go is refused and `abort_reason` says why. `go` already on at arm doesn't count.
- **ASCENT** → **HOVER** → **DESCENT** → **LANDED**: the setpoint ramps up at `v_up` from where the hopper is, holds the hop height for `hover_ms`, ramps down at `v_down` to 10 cm below the ground (so the controller keeps pushing down), and the valve shuts once the hopper is resting on the ground. LANDED holds the valve shut until the ground disarms.
- **ABORT** (go dropped, flight timeout, too high): controlled descent at 0.4 m/s; valve shut if it hasn't landed in 5 s.
- Sensor loss (IMU, or no laser range for 200 ms) or servo power loss: valve shut at once, straight to LANDED. Without them a controlled descent isn't possible.
- The hop profile (height, climb rate, descent rate, hover time) is sent in every command, clamped by the hopper (0.1–0.8 m, 0.05–0.5 m/s, 0.05–0.3 m/s, 0–10 s) and latched at go. The abort behaviour and those limits are constants in `flight.cpp`.
- Test without hardware: `test/flight_test.cpp` flies the phases and the real controllers on a 1-D vehicle model.

Each module is a `.h`/`.cpp` pair, so you only edit the files you own. To add one (e.g. an IMU driver), add `src/drivers/imu.h` and `src/drivers/imu.cpp`, then include it by relative path where it's used: `#include "src/drivers/imu.h"` from `hopper.ino`, or `#include "../drivers/imu.h"` from another folder in `src/`. The Arduino IDE compiles everything under `src/` automatically.

### Sending telemetry from the control loop

```cpp
#include "../telemetry/telemetry.h"

telemetry_data_t data = { ... };
telemetry_push(&data);  // non-blocking, never takes a lock
```

`telemetry_push` must only be called from one task. To change what is sent, edit `telemetry_data_t` in `src/protocol.h`. **`hopper/src/protocol.h` and `ground/src/protocol.h` must stay identical**, and the ground station's `sprint 2/ground_station/protocol.py` (`TELEMETRY_FMT`, `CMD_FMT`) must match.

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
    │   ├── pins.h      LED pins
    │   ├── lights.*    Green LED (D6), red LED (D5)
    │   └── radio.*     Same ESP-NOW driver as the hopper's
    ├── usb/
    │   └── usb_link.*  Framed packets to and from the laptop
    └── bridge/
        └── bridge.*    Hopper <-> laptop relay, lights
```

Over USB each packet is framed as `0xAA 0x55 <len> <packet> <checksum = sum of packet bytes mod 256>`. Nothing else is printed, so don't use the Serial Monitor on this board (close it before connecting the dashboard).

- E-stop: digital only, from the dashboard (E-stop button or Esc), sent as `estop = 1` in every command. There is no physical switch on the ground board. If the dashboard or laptop stops sending, the hopper goes FAILSAFE on link loss (150 ms).
- Lights: GREEN = hearing the hopper. RED solid = hopper ARMED, RED blinking = hopper FAILSAFE. Both on for 0.5 s at power-up (lamp test).

Setup: same as the hopper (no extra libraries). Fill in `HOPPER_MAC` in `ground/src/protocol.h`, open `ground/ground.ino` and upload. Then open the dashboard in Chrome or Edge and press Connect.
