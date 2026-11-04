# hopper-sensors

ESP-IDF starter project for the lunar hopper sensor suite. Currently drives one
VL53L0X time-of-flight rangefinder, structured so that adding the IMU and load
cells does not touch `main.c`.

Written in **C** (ESP-IDF is a C framework, unlike Arduino which is C++). Every
file is heavily commented for learning purposes — see
`../../../docs/vl53l0x-esp32.md` for the sensor theory and an Arduino-to-ESP-IDF
language appendix.

## Requirements

- **ESP-IDF v5.2 or newer.** The project uses the `driver/i2c_master.h` API
  introduced in v5.2. On v5.1 or earlier it will not compile — most tutorials
  online still use the old `driver/i2c.h` API, which is not interchangeable.
- VSCode with the Espressif IDF extension, or `idf.py` on the command line.

## Layout

```
hopper-sensors/
├── CMakeLists.txt                  project root
├── main/
│   └── main.c                      app_main: bus setup, sensor table, loop
└── components/
    ├── sensor_common/
    │   └── include/sensor_iface.h  the generic sensor interface (vtable)
    ├── i2c_bus/                    one shared I2C master bus + scanner
    └── vl53l0x/
        ├── vl53l0x.c               register-level driver (~1400 lines)
        ├── vl53l0x_iface.c         sensor_iface adapter + median filter
        └── include/
```

Three layers, each replaceable without disturbing the others:

| Layer | Knows about | Does not know about |
|---|---|---|
| `vl53l0x.c` | the chip's registers | the sensor interface |
| `vl53l0x_iface.c` | both | the application |
| `main.c` | the sensor interface | any specific sensor |

## Board selection

`main.c` has a `BOARD` define at the top. Set it, and the pins follow:

```c
#define BOARD   BOARD_NANO_ESP32     /* or BOARD_ESP32_DEVKIT */
```

Pin numbers throughout are **raw ESP32 GPIO numbers**, not Arduino labels.
This is the biggest single gotcha coming from the Arduino IDE: there, `A4` is
a label the core silently remaps to a GPIO. ESP-IDF has no core and does no
remapping — it wants the real number from the schematic.

## Wiring

### Arduino Nano ESP32 (ESP32-S3) — the default

| Breakout | Board silkscreen | Raw GPIO |
|---|---|---|
| `VIN` | `3V3` | — |
| `GND` | `GND` | — |
| `SDA` | `A4` | 11 |
| `SCL` | `A5` | 12 |
| `XSHUT` | `D2` | 5 |
| `GPIO1` | not used | — |

Connect to the pins **labelled** `A4`/`A5`/`D2`. The numbers in `main.c` are
already the raw GPIOs those labels map to.

> **Do not use `SDA=21, SCL=22` on this board.** The ESP32-S3's GPIO numbering
> skips 22 through 25 — GPIO 22 does not exist. `gpio_config()` rejects it.
> A `_Static_assert` in `main.c` now catches this at build time.
>
> Also avoid GPIO 26–32 on the S3: they are wired to the module's SPI flash
> and PSRAM, and using them can stop the board booting.

The Nano ESP32 runs its GPIO at 3.3 V, same as any ESP32. Power the breakout
from `3V3`, not `VBUS`/`5V`.

### Classic ESP32 DevKitC

| Breakout | GPIO |
|---|---|
| `SDA` | 21 |
| `SCL` | 22 |
| `XSHUT` | 5 |

Set `BOARD` to `BOARD_ESP32_DEVKIT`.

---

Wiring `XSHUT` is recommended on either board. The ESP32 reboots on every
flash upload while the sensor keeps its power, and a sensor left mid-measurement
does not re-initialise cleanly. `XSHUT` gives the driver a known starting point.

## Build

### Arduino Nano ESP32

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Note `/dev/ttyACM0`, not `ttyUSB0`: the Nano ESP32 has **no USB-to-UART bridge
chip**. It uses the ESP32-S3's native USB, so it enumerates as a CDC-ACM
device. Two consequences:

- **`sdkconfig` must route the console to USB.** Run `idf.py menuconfig` and
  set *Component config → ESP System Settings → Channel for console output* to
  **USB Serial/JTAG Controller**. Without this, `idf.py monitor` connects and
  shows nothing — the logs are going to physical UART0 pins instead.
- **The port disappears on reset.** The USB device is implemented by the chip,
  so it re-enumerates each time the chip reboots. If flashing fails, double-tap
  the reset button to force the ROM bootloader, then retry.

On WSL you also need the device passed through with `usbipd` before
`/dev/ttyACM0` appears.

### Classic ESP32 DevKitC

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

In VSCode: the target selector is in the status bar; build/flash/monitor are
the ⚙/⚡/🖥 icons.

### A note on flashing this board outside the Arduino IDE

The Nano ESP32 ships with an Arduino bootloader. Flashing it with `idf.py`
replaces that, which is expected and fine — but to go back to using the Arduino
IDE with this board you will need to restore it (double-tap reset, then
*Tools → Burn Bootloader*, or re-flash any Arduino sketch via DFU). Worth
knowing before you flash, not after.

## Expected output

```
I (310) main: hopper sensor suite starting
I (320) i2c_bus: bus up on SDA=11 SCL=12
I (330) i2c_bus: scanning bus...
I (420) i2c_bus:   found device at 0x29
I (430) i2c_bus: scan complete: 1 device(s)
I (440) vl53l0x: found VL53L0X at 0x29
I (490) vl53l0x: initialised: budget 33000 us, offset 0 mm, 12 ref SPADs
I (500) vl53l0x: continuous mode started (period 0 ms)
I (510) vl53l0x_if: [altimeter] ready
I (520) main: 1/1 sensors online
I (770) main: alt  0.342 m | age   8 ms | loop  20 ms | n=12
```

If the scan finds nothing, stop and fix that before looking at any other code:
it is a power or wiring fault, not software.

## Calibration

Offset calibration is the one you need; a 10–20 mm raw offset is normal.

1. Place a flat white target at exactly 100 mm.
2. Uncomment the `vl53l0x_calibrate_offset` block in `app_main`.
3. Flash, read the offset off the console.
4. Put it in `TOF_OFFSET_MM` and comment the block back out.

Better: store it in NVS so it survives a reflash. Crosstalk calibration is not
implemented — you only need it behind cover glass. See notes section 10.

## Adding another sensor

`main.c` does not change. The steps:

1. Create `components/<name>/` with the same two-file split — `<name>.c` for
   registers, `<name>_iface.c` for the vtable — and a `CMakeLists.txt`.
2. Add an entry to `sensor_id_t` in `main.c`, above `SENSOR_COUNT`.
3. Declare its context struct next to `g_tof_ctx`.
4. Call `<name>_sensor_create(&g_sensors[SENSOR_X], ...)` in `sensors_setup()`.
5. Add the component to `REQUIRES` in `main/CMakeLists.txt`.

The polling loop, error handling, and telemetry pick it up automatically.

## Tuning

`tof_cfg.profile` in `main.c`:

| Profile | Budget | Range | Notes |
|---|---|---|---|
| `HIGH_SPEED` | 20 ms | shortest | 50 Hz, least lag |
| `DEFAULT` | 33 ms | ~1.2 m | balanced |
| `LONG_RANGE` | 33 ms | ~2 m | ambient-sensitive |
| `HIGH_ACCURACY` | 200 ms | ~1.2 m | 5 Hz, best precision |

`VL53L0X_FILTER_TAPS` in `vl53l0x_iface.h` sets the median window. 5 taps at a
33 ms budget is roughly 80 ms of group delay — a lot inside a hopper loop.
Consider 3 taps with `HIGH_SPEED` if phase margin is tight.

## Interrupt mode

The code polls `RESULT_INTERRUPT_STATUS` over I²C, which costs one register
read per loop iteration and is fine at 50 Hz. The chip also drives `GPIO1`
(open-drain, active low) when a sample is ready. To use it, wire `GPIO1`,
`gpio_install_isr_service`, register a handler with `IRAM_ATTR` that only sets
a flag, and gate `sensor_poll` on it. The driver already configures the pin.

Keep I²C out of the ISR. The ESP-IDF I²C driver takes a mutex; calling it from
interrupt context deadlocks.

## Limits worth designing around

- **~2 m in darkness, ~0.8–1.2 m in daylight.** A landing sensor, not an
  altimeter. Above ~1 m you need a VL53L1X (4 m) or a VL53L5CX (4 m plus an
  8×8 zone array).
- **25° field of view.** Spot diameter ≈ 0.44 × distance, so at 1 m you are
  reading a 44 cm patch — and the *nearest* thing in that cone wins. Legs,
  tether, and test-stand hardware will shorten your altitude.
- **It measures slant range.** `altitude ≈ range × cos(tilt)`. At 20° you read
  6% high. Needs the IMU.
- **Latency.** The reading averages over the timing budget, so its true epoch
  is ~budget/2 before the timestamp, plus median group delay. Model it rather
  than tuning around it.

## Verification status

The code compiles clean (`-Wall -Wextra -Wshadow -Wsign-compare`, gnu99 and
gnu17) and its logic was tested on a host against a simulated VL53L0X register
file: 48 checks covering init, timing-budget arithmetic and clamping, VCSEL
period validation, offset application including the negative-clamp path,
sanity-window and status-code rejection, continuous-mode reads, health
counters, null guards, and offset calibration. The median filter was verified
against an independent reference over 4000 samples with injected outliers
(exact agreement), with steady-state lag matching the predicted
`(taps/2) × step`. All of it passes under ASan and UBSan.

**None of this has run on hardware.** Host tests cannot validate the
undocumented register sequences, real I²C timing, or the analog front end.
Expect to debug on first flash, and start with the bus scan.
