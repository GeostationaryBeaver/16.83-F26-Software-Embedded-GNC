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
    │   ├── control.*   Commands, link timeout, altitude filter, telemetry. GNC controller hook
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
- GNC: the controller goes in `valve_demand()` in `src/control/control.cpp`. Today it returns the ground's manual valve command.

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
