# VL53L0X bring-up test

Minimal Arduino sketch to confirm the VL53L0X is wired correctly and working,
before moving to the ESP-IDF project in `../esp-idf/hopper-sensors/`.

Target: **Arduino Nano ESP32**, Arduino IDE. This is C++ (Arduino framework),
not the C used by ESP-IDF.

## Wiring — 4 wires

| Breakout | Nano ESP32 (silkscreen) |
|---|---|
| `VIN` | **`3V3`** |
| `GND` | `GND` |
| `SDA` | **`A4`** |
| `SCL` | **`A5`** |
| `XSHUT` | leave unconnected |
| `GPIO1` | leave unconnected |

On the Nano form factor there are no pins marked SDA/SCL — `A4` and `A5` *are*
the I²C pins.

**Use `3V3`, not `5V`.** The breakout's regulator tolerates 5 V, but its I²C
pull-ups would then sit at 5 V against a 3.3 V ESP32. (The Nano ESP32's `5V`
pin is also disconnected by default — it needs a solder jumper bridged — so
this is easy to get wrong in the confusing direction.)

`XSHUT` is pulled high on the breakout, so the sensor runs with it floating.
Fewer wires, fewer faults. Wire it later for the ESP-IDF project.

## If you edit this on WSL but run the IDE on Windows

The Windows Arduino IDE cannot reliably open sketches across the `\\wsl$\`
boundary. Instead of reporting an error it creates a blank, date-named sketch,
which looks like the file failed to exist.

Copy the sketch folder into the Windows sketchbook and open it from there
(*File → Sketchbook*). Note that Windows often redirects `Documents` into
OneDrive, so the sketchbook is typically:

```
C:\Users\<you>\OneDrive\Documents\Arduino\
```

The repo copy and the sketchbook copy are then two separate files; keep track
of which one you are editing.

## Setup

1. **Board package** — do this first, and do not skip it.

   *Tools → Board → Boards Manager*, search `esp32`, install
   **"Arduino ESP32 Boards" by Arduino**. It is a ~200 MB toolchain download,
   so allow a few minutes.

   > Two packages match `esp32`. Install **"Arduino ESP32 Boards" by
   > Arduino**, which carries the `arduino_nano_nora` variant and its pin
   > remapping. The other, *"esp32" by Espressif Systems*, is the generic
   > core: it will build, but it lists generic S3 dev boards instead of this
   > one and you lose the `A4` → GPIO 11 label mapping this sketch relies on.

   **If you skip this**, the IDE silently compiles for whatever board was
   last selected — typically an AVR Uno or Nano — and you get:

   ```
   error: no matching function for call to
          'TwoWire::begin(const uint8_t&, const uint8_t&)'
   ```

   The tell is the include path in the error: `hardware\avr\...\Wire.h`.
   AVR's `Wire.begin()` takes no arguments because its I²C pins are fixed in
   silicon and cannot be reassigned. `Wire.begin(sda, scl)` is an ESP32-core
   signature, so that error means the ESP32 core is not being used — not that
   anything is wrong with the sketch.

2. **Board:** *Tools → Board → Arduino ESP32 Boards → Arduino Nano ESP32*

3. **Library:** *Sketch → Include Library → Manage Libraries*, search
   `VL53L0X`, install **"VL53L0X" by Pololu**. (Not the Adafruit one — this
   sketch uses Pololu's API.)
4. **Port:** *Tools → Port* — it appears as `/dev/ttyACM*` on Linux or a COM
   port on Windows. The Nano ESP32 uses native USB, so the port vanishes and
   re-enumerates on every reset. That is normal.
5. Upload, then open *Tools → Serial Monitor* at **115200 baud**.

## What you should see

```
========================================
 VL53L0X bring-up test
 Arduino Nano ESP32
========================================

--- I2C bus scan ---
  device at 0x29  <-- VL53L0X
  1 device(s) found.

--- sensor init ---
  init() ok
  continuous mode started

--- ranging ---
Wave your hand over the sensor. Expect roughly
50-1200 mm. 8190 means no target / out of range.

247 mm   ####
198 mm   ###
---- mm   (no target in range)
512 mm   ##########
```

Wave your hand over the lens; the number and bar should track it.

## Reading a failure

The sketch runs in two phases on purpose, because they fail for different
reasons.

### Scan finds nothing

Electrical. The sensor is not powered or not connected. Check in this order:

1. `VIN` on `3V3` — not `5V`, not `VBUS`
2. `GND` connected
3. `SDA` on `A4`, `SCL` on `A5` — and **not swapped**
4. Jumper wires seated, and not broken internally (a surprisingly common
   failure; swap them before anything else)

### Scan finds a device at a different address

Something is on the bus but it is not at `0x29`. Either it is not a VL53L0X,
or it is a module variant with a different default address. The sketch prints
whatever it finds, which tells you which.

### Scan finds `0x29` but `init()` fails

The wiring is fine — the chip acknowledged. Usually:

- **Marginal power.** Try a different `3V3` jumper.
- **400 kHz too fast for your wiring.** Long or unshielded jumpers can fail at
  400 kHz while working at 100 kHz. Change the `Wire.setClock(400000)` line in
  the init section to `100000` and retry. If that fixes it, shorten the wires
  rather than leaving it slow.
- **A counterfeit or damaged module.** Common with cheap breakouts.

### Readings are stuck at ~8190

No target in range, or the lens is obstructed. Check nothing is touching the
glass, and aim it at a wall 10–50 cm away.

### Readings are implausibly short

Something is inside the 25° cone that you are not aiming at — the sensor
reports the *nearest* strong return, not what is on axis. At 1 m the cone is
~44 cm across. Also check the lens is clean: a smudge or flux residue acts as
cover glass and biases readings short.

## Next step

Once this passes, the sensor and wiring are confirmed. Move to
`../esp-idf/hopper-sensors/` for the real driver — and note that flashing with
`idf.py` replaces the Arduino bootloader, so do this test first.

See `../../docs/vl53l0x-esp32.md` for the sensor theory, the full
label-to-GPIO table (Appendix C), and the Arduino-to-ESP-IDF language notes
(Appendix A).

## Verification status

The sketch compiles clean under `g++ -Wall -Wextra` against stub Arduino
headers, and its display logic (bar graph scaling, out-of-range sentinel
handling) was verified exhaustively over the full `uint16_t` output range.

**It has not run on hardware** — that is what it is for. Stub headers cannot
validate the real Pololu library API or I²C timing.
