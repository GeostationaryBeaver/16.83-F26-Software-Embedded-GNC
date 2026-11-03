# VL53L0X Time-of-Flight Rangefinder on ESP32

**Reference notes for the lunar hopper GNC stack** — how the sensor physically
works, how to wire and drive it from an ESP32, how to calibrate it, and what its
limits mean for closed-loop altitude control.

---

## Contents

1. [What the part is](#1-what-the-part-is)
2. [How a measurement is physically made](#2-how-a-measurement-is-physically-made)
3. [Performance envelope](#3-performance-envelope)
4. [What degrades the reading](#4-what-degrades-the-reading)
5. [The register interface](#5-the-register-interface)
6. [Wiring to the ESP32](#6-wiring-to-the-esp32)
7. [Code: minimal bring-up](#7-code-minimal-bring-up)
8. [Code: continuous mode with interrupt and filtering](#8-code-continuous-mode-with-interrupt-and-filtering)
9. [Range profiles](#9-range-profiles)
10. [Calibration](#10-calibration)
11. [Implications for the hopper](#11-implications-for-the-hopper)
12. [Quick reference](#12-quick-reference)
13. [Appendix A: Arduino C++ vs ESP-IDF C](#appendix-a-arduino-c-vs-esp-idf-c)
14. [Appendix B: the ESP-IDF starter project](#appendix-b-the-esp-idf-starter-project)
15. [Appendix C: Arduino Nano ESP32 notes](#appendix-c-arduino-nano-esp32-notes)

---

## 1. What the part is

"Qoroos VL53L0X" is a reseller label on a generic breakout (GY-530 /
GY-VL53L0XV2 class) carrying ST's **VL53L0X** chip. The chip is the interesting
part; the board around it is an LDO, decoupling, I²C pull-ups, and a 6-pin
header.

Inside the 4.4 x 2.4 x 1.0 mm module there are two optical cavities under a
single glass lid:

- **Emitter** — a 940 nm VCSEL (vertical-cavity surface-emitting laser),
  Class 1 eye-safe, driven in short pulses.
- **Receiver** — an array of **SPADs** (single-photon avalanche diodes) plus the
  timing ASIC, behind an IR bandpass filter.

940 nm sits in a water-absorption notch in the solar spectrum, so there is less
sunlight there to compete with, and it is invisible to humans.

This is **direct time-of-flight**, not triangulation. That is the key difference
from the older Sharp IR sensors: accuracy does not collapse with distance, and
it is largely independent of target colour and reflectance. Reflectance affects
*range* and *noise*, not the distance scale.

---

## 2. How a measurement is physically made

### Photon timing

The VCSEL fires a pulse. A photon that returns and strikes a SPAD triggers an
avalanche — a single photon produces a measurable current spike, time-stamped
relative to emission. One photon gives one arrival time, with huge variance. So:

1. The VCSEL fires **thousands of pulses** over the measurement window.
2. Each SPAD event is binned by arrival time into a **histogram**.
3. Ambient light (sunlight, room lighting) produces a flat, uncorrelated
   pedestal across all bins. The target return produces a peak.
4. The ASIC subtracts the pedestal and finds the peak centroid, then
   `distance = c * t_peak / 2`.

Ambient light is uncorrelated noise while the signal accumulates coherently, so
SNR grows roughly as the square root of the pulse count. That is exactly why
integration time trades against range.

### Timing budget and convergence

You give the sensor a **timing budget** (default 33 ms, minimum 20 ms,
"high accuracy" 200 ms). Internally the measurement splits into phases:

| Phase | Purpose |
|---|---|
| VHV / phase calibration | Re-trims SPAD breakdown bias, which drifts with temperature |
| Pre-range | Coarse pass with longer VCSEL period — finds roughly where the target is and how much signal there is |
| Final range | Precise pass, integration window steered by the pre-range result |

There is also a **convergence** mechanism: the sensor keeps accumulating until
it reaches a target SNR *or* exhausts the budget. A bright close target
converges early (fast, accurate); a dark far target burns the whole budget and
may still fail.

> **Consequence:** measurement duration is *data-dependent* up to the budget
> ceiling. This matters if you are counting on fixed loop timing.

### What comes out

Two 8-bit registers give **distance in millimetres** (integer, no sub-mm
resolution), plus a **range status** code and signal/ambient rate counters.

Status is what tells you the number is trustworthy. The codes you will actually
see are *sigma too high*, *signal too low*, *phase fail* (wrap-around
ambiguity), and *hardware fail*.

---

## 3. Performance envelope

| Property | Value |
|---|---|
| Max range | ~2 m in darkness, **~0.8–1.2 m in bright or sunlit conditions** |
| Accuracy | ±3% typical at mid-range; ±5% at long range or low reflectance |
| Resolution | 1 mm |
| Field of view | **~25° full cone** |
| Rate | up to 50 Hz (20 ms budget) |
| Min range | works from ~0 mm, but crosstalk-sensitive below ~30 mm |
| Interface | I²C, 7-bit address `0x29`, up to 400 kHz |
| Wavelength | 940 nm, Class 1 |

### The field of view matters more than people expect

The illuminated spot diameter is approximately `0.44 x distance`. At 1 m you are
interrogating a **44 cm patch**, and the reported number is dominated by the
*strongest* return in that cone — usually the *nearest* object, not the centre
of the aimed axis.

For a hopper that means a leg, a tether, a test-stand upright, or a wall inside
the cone will shorten your "altitude" reading.

---

## 4. What degrades the reading

- **Ambient IR** — sunlight raises the histogram pedestal and kills far range.
- **Low-reflectance or angled targets** — less signal returns. Beyond roughly
  60° off-normal incidence the return mostly reflects specularly away.
- **Cover glass** — internal reflection ("crosstalk") adds a fixed near-field
  return that biases you short. This **must** be calibrated out.
- **Temperature** — handled by per-measurement VHV recalibration, but a
  cold-start offset shift is normal.
- **Multiple sensors firing simultaneously** — they see each other's photons.
  Interleave them via XSHUT or accept the noise.
- **A dirty lid** — flux residue, a smudge, or conformal coating over the glass
  acts as cover glass and biases you short.

---

## 5. The register interface

The chip is I²C at 7-bit address **`0x29`**. The datasheet quotes `0x52`, which
is the same address shifted left for the write bit — this trips people up
constantly.

In principle ranging is simple:

| Register | Name | Use |
|---|---|---|
| `0xC0` | `IDENTIFICATION_MODEL_ID` | Must read `0xEE` — bring-up sanity check |
| `0x00` | `SYSRANGE_START` | Write `0x01` to start a single measurement |
| `0x13` | `RESULT_INTERRUPT_STATUS` | Poll low 3 bits for "ready" |
| `0x14` | `RESULT_RANGE_STATUS` | `+10`/`+11` hold the 16-bit range in mm |
| `0x0B` | `SYSTEM_INTERRUPT_CLEAR` | Write `0x01` to clear and re-arm |

**In practice, ST never published the register map.** Everything else — the ~80
undocumented "tuning" registers, SPAD reference management, VHV/phase
calibration sequencing, VCSEL pulse-period programming, timing-budget
arithmetic — exists only as ST's C API, or as Pololu's clean-room
reimplementation of it.

> **Use a library.** Hand-rolling init is a multi-week detour with no payoff.

### Library options

| Library | Character |
|---|---|
| **`VL53L0X` by Pololu** | Small, readable, no HAL layer. Exposes timing budget, VCSEL period, and signal-rate limit directly. **Recommended**, and what the code below uses. |
| `Adafruit_VL53L0X` | Wraps ST's official API. Heavier, more RAM, but gives you ST's calibration routines and canned range profiles. |
| ESP-IDF native | No first-party component. Port ST's API over `i2c_master`, or use a community wrapper. Only worth it if you are already off Arduino. |

---

## 6. Wiring to the ESP32

Check your specific board first, but the GY-530-family pinout is:

| Breakout | ESP32 (classic DevKit) | Nano ESP32 (S3) | Notes |
|---|---|---|---|
| `VIN` | **3V3** | **3V3** | The board has an LDO and tolerates 5 V, but 3V3 keeps the I²C pull-ups at ESP32 logic level. Use 3V3. |
| `GND` | `GND` | `GND` | Common ground with the ESP32 **and** the valve driver |
| `SCL` | `GPIO 22` | `A5` = GPIO 12 | 400 kHz |
| `SDA` | `GPIO 21` | `A4` = GPIO 11 | |
| `GPIO1` | `GPIO 4` (optional) | `D3` = GPIO 6 | Interrupt out — **open-drain, active low**, needs a pull-up |
| `XSHUT` | `GPIO 5` (optional) | `D2` = GPIO 5 | **Active-low** hardware reset/standby — drive low to disable |

The Nano ESP32 column gives silkscreen label and raw GPIO, because they differ
and ESP-IDF wants the raw number. See [Appendix C](#appendix-c-arduino-nano-esp32-notes).

### Gotchas

- **On ESP32-S3 and -C3 the default I²C pins differ**, and on the S3
  `GPIO 22` does not exist at all (its numbering skips 22–25), so the classic
  `SDA=21, SCL=22` pairing cannot work there. Always call
  `Wire.begin(sdaPin, sclPin)` explicitly rather than trusting the default.
- **The chip core runs at 2.8 V.** On a bare chip, `GPIO1` and `XSHUT` are 2.8 V
  logic and `XSHUT` must be actively driven — there is no internal pull-up.
  Breakouts usually add a pull-up and a buffered path, but verify yours with a
  meter before assuming 3.3 V drive into `XSHUT` is safe.
- **The I²C address is not persistent.** You set it in software after every
  power-up. To run two or more sensors on one bus you must hold all but one in
  `XSHUT`, bring one up, reassign its address, then release the next. There is
  no other way.

---

## 7. Code: minimal bring-up

Start here to prove the hardware. Polled and blocking, `Serial.print` only, no
control logic.

```cpp
#include <Wire.h>
#include <VL53L0X.h>

static const int PIN_SDA = 21;
static const int PIN_SCL = 22;

VL53L0X sensor;

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  sensor.setTimeout(500);
  if (!sensor.init()) {              // reads model ID 0xEE, loads tuning, calibrates
    Serial.println("VL53L0X not found - check wiring and address 0x29");
    while (true) delay(1000);
  }

  sensor.setMeasurementTimingBudget(33000);   // microseconds
}

void loop() {
  uint16_t mm = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial.println("timeout");
  } else {
    Serial.println(mm);
  }
  delay(50);
}
```

If `init()` fails the sensor is not talking. Scan the bus and confirm `0x29`
appears before changing anything else.

---

## 8. Code: continuous mode with interrupt and filtering

For a control loop you want three things the sketch above does not give you:
**no blocking**, **deterministic cadence**, and **rejected bad samples**.

Continuous mode has the sensor range back-to-back on its own clock and assert
`GPIO1` when a result is ready; you only latch it.

```cpp
#include <Wire.h>
#include <VL53L0X.h>

static const int PIN_SDA   = 21;
static const int PIN_SCL   = 22;
static const int PIN_INT   = 4;    // sensor GPIO1, active low
static const int PIN_XSHUT = 5;    // active low

static const uint32_t TIMING_BUDGET_US = 33000;
static const uint16_t RANGE_MIN_MM     = 30;     // below this, crosstalk dominates
static const uint16_t RANGE_MAX_MM     = 1600;   // be honest about the ceiling

VL53L0X sensor;

volatile bool sampleReady = false;
void IRAM_ATTR onRangeReady() { sampleReady = true; }

// 5-tap median: kills the isolated dropouts that a mean would smear
static uint16_t ring[5];
static uint8_t  ringIdx = 0;
static bool     ringPrimed = false;

static uint16_t median5() {
  uint16_t s[5];
  memcpy(s, ring, sizeof(s));
  for (uint8_t i = 1; i < 5; i++)
    for (uint8_t j = i; j > 0 && s[j] < s[j - 1]; j--) {
      uint16_t t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
    }
  return s[2];
}

struct Range {
  bool     valid;
  uint16_t raw_mm;
  uint16_t filtered_mm;
};

void setup() {
  Serial.begin(115200);

  // Hardware reset so a warm MCU reboot cannot leave the sensor mid-measurement
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(10);                       // ~1.2 ms boot, be generous

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("FATAL: VL53L0X init failed");
    while (true) delay(1000);
  }

  // --- Long-range profile: trades ambient-light immunity for reach ---
  sensor.setSignalRateLimit(0.1);                                  // default 0.25 MCPS
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,   18);  // default 14
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);  // default 10

  sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);

  pinMode(PIN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), onRangeReady, FALLING);

  sensor.startContinuous();        // sensor now paces itself off the timing budget
}

// Non-blocking: returns valid == false when there is nothing new
Range pollRange() {
  Range r = {false, 0, 0};

  if (!sampleReady) return r;
  sampleReady = false;

  uint16_t mm = sensor.readRangeContinuousMillimeters();   // also clears the interrupt
  if (sensor.timeoutOccurred()) return r;

  r.raw_mm = mm;

  // 8190/8191/65535 are the out-of-range sentinels; then apply our own window
  if (mm >= 8190 || mm < RANGE_MIN_MM || mm > RANGE_MAX_MM) return r;

  ring[ringIdx++] = mm;
  if (ringIdx >= 5) { ringIdx = 0; ringPrimed = true; }
  if (!ringPrimed) return r;

  r.filtered_mm = median5();
  r.valid = true;
  return r;
}

void loop() {
  Range r = pollRange();
  if (r.valid) {
    Serial.printf("%lu,%u,%u\n", millis(), r.raw_mm, r.filtered_mm);
  }

  // ... GNC / valve update runs here, never blocked by the sensor ...
}
```

### Why the code looks like that

- **The ISR only sets a flag.** I²C in an ISR will deadlock or crash you —
  `Wire` is not interrupt-safe and the ESP32 I²C driver takes a mutex.
- **`readRangeContinuousMillimeters()` clears the interrupt**, which re-arms
  `GPIO1`. Skip the read and you never get another edge.
- **The `XSHUT` reset in `setup()`** matters on ESP32 specifically: the MCU
  reboots on upload or watchdog far more often than the sensor loses power, and
  a sensor left in continuous mode will not init cleanly.
- **Median before mean.** VL53L0X failures are isolated gross outliers, not
  Gaussian noise. A median rejects them; an averaging filter propagates them.
- **Range status filtering is not optional.** The sensor will happily hand you a
  confident-looking number with a "sigma too high" flag on it.

---

## 9. Range profiles

| Goal | Budget | VCSEL pre/final | Signal limit |
|---|---|---|---|
| High speed (~50 Hz) | 20 ms | 14 / 10 | 0.25 |
| Default | 33 ms | 14 / 10 | 0.25 |
| Long range (~2 m) | 33 ms | 18 / 14 | 0.1 |
| High accuracy | 200 ms | 14 / 10 | 0.25 |

Long-range periods combine with a 200 ms budget for the best far-field
precision, at 5 Hz.

---

## 10. Calibration

Three calibrations exist, in the order they matter:

### 1. Offset — you definitely need this

Put a flat 88%-reflectance white target at exactly 100 mm, average ~50 readings,
and store `offset = 100 - measured`. Subtract it thereafter. A 10–20 mm raw
offset is normal.

### 2. Crosstalk — only behind cover glass

Measure a target at a known *far* distance (~400 mm) with the glass fitted; the
induced short-bias gives the crosstalk compensation rate.

> Skipping this with a window installed is the single most common cause of
> "it reads 200 mm no matter what".

### 3. Reference SPAD / VHV — automatic

Done inside `init()` on every power-up. Leave it alone.

Store the offset in NVS / `Preferences` rather than recompiling it, and re-check
it if you change the lens aperture, the mounting, or the operating temperature
range.

---

## 11. Implications for the hopper

### The 2 m ceiling is hard, and it is ~1 m in daylight

This is a landing/touchdown and low-hover sensor, not an altimeter. If the
hopper's trajectory goes above ~1 m you need a second sensor:

| Part | Range | Note |
|---|---|---|
| VL53L1X | ~4 m | Drop-in successor, same approach |
| VL53L5CX | ~4 m | Adds an 8x8 zone array — also solves the cone problem |
| Radar / laser rangefinder | beyond that | Different class of part entirely |

Plan the handover logic now rather than later.

### It measures slant range, not altitude

Under tilt `θ`, `altitude ≈ range * cos(θ)`. With IMU attitude available, apply
that correction — at 20° of tilt you read 6% high otherwise, and the cone is
also sampling ground you are not above.

### Latency eats phase margin

The reported range is an average over the timing budget, so the effective
measurement delay is roughly **half the budget plus the I²C transfer** — about
20 ms at a 33 ms budget. The 5-tap median adds another ~2.5 samples of group
delay, roughly 80 ms.

That is a lot of lag for a hopper. Consider a 20 ms budget with a 3-tap median,
and model the residual delay explicitly in the controller rather than tuning
around it.

### Thruster plume, dust, and the test stand

The 25° cone will see stand hardware and tether, and downwash-lofted dust gives
spurious short returns. Mount the sensor looking at clear floor, shroud it from
the plume, and treat a sudden short jump with no corroborating IMU acceleration
as a reject.

---

## 12. Quick reference

| | |
|---|---|
| I²C address | `0x29` (7-bit) — datasheet's `0x52` is the same, shifted |
| Model ID register | `0xC0`, must read `0xEE` |
| Supply | 3V3 to breakout `VIN` |
| Default ESP32 I²C | `GPIO 21` SDA, `GPIO 22` SCL (classic ESP32 only) |
| Nano ESP32 (S3) I²C | `A4` = GPIO 11 SDA, `A5` = GPIO 12 SCL — **GPIO 22 does not exist on S3** |
| `XSHUT` | Active low, no internal pull-up on bare chip |
| `GPIO1` interrupt | Open-drain, active low, needs pull-up |
| Out-of-range sentinels | `8190`, `8191`, `65535` |
| Library | `VL53L0X` by Pololu |
| Minimum timing budget | 20 ms |
| Spot diameter | ~`0.44 x distance` |

### Bring-up checklist

- [ ] `VIN` on 3V3, grounds common with MCU and valve driver
- [ ] I²C scan shows `0x29`
- [ ] `init()` returns true
- [ ] Static target at 100 mm reads plausibly; offset recorded
- [ ] Crosstalk calibrated if behind glass
- [ ] Continuous mode running, interrupt firing, no timeouts
- [ ] Range window and status filtering rejecting bad samples
- [ ] Offset persisted to `Preferences`
- [ ] Measured end-to-end latency folded into the control model

---

## Appendix A: Arduino C++ vs ESP-IDF C

### The short version

Two separate facts, and it is worth keeping them apart:

| | Language | Framework |
|---|---|---|
| **Arduino** (`.ino`, Arduino IDE) | **C++** | Arduino core |
| **ESP-IDF** (`.c`, VSCode ESP-IDF extension) | **C** | ESP-IDF |

So: every Arduino sketch you have ever written was already C++ — you have not
been writing some separate "Arduino language". But **ESP-IDF is a C framework**,
and that is a real language change, not just a different library.

> **Earlier in these notes the code listings in sections 7 and 8 are C++**
> (Arduino framework). The driver in Appendix B is **C** (ESP-IDF). They do the
> same job in two different languages. Read this appendix to see which
> constructs belong to which.

### What an `.ino` file actually is

The Arduino IDE is a thin wrapper around `g++`. On compile it takes your `.ino`
and makes it a valid C++ file by:

1. Prepending `#include <Arduino.h>`
2. Auto-generating forward declarations for all your functions
3. Concatenating any additional `.ino` tabs (alphabetically, after the main one)

Then handing the result to the same compiler that builds any C++ project. That
is why `pinMode()`, `Serial.println()`, and `String` are all C++.
`MAC_35_driver_v1.ino` is C++ with `const`, `bool`, function return types, and
casts already in it.

---

### Part 1 — Constructs in the Arduino listings (C++)

These appear in sections 7 and 8 above, and not in the Sprint 1 valve driver.

#### A1. Fixed-width integer types

The valve driver used `int`, `float`, `unsigned long`. The sensor code uses
`uint16_t`, `uint8_t`, `uint32_t`.

| Type | Meaning | Range |
|---|---|---|
| `uint8_t` | unsigned 8-bit | 0 … 255 |
| `uint16_t` | unsigned 16-bit | 0 … 65,535 |
| `uint32_t` | unsigned 32-bit | 0 … 4,294,967,295 |
| `int8_t`, `int16_t`, `int32_t` | signed versions | e.g. −32,768 … 32,767 |

Not new types — guaranteed-size aliases. Why it matters moving from the Mega to
the ESP32:

```cpp
int x;   // 16-bit on Arduino Mega (max 32767)
         // 32-bit on ESP32      (max 2147483647)
```

Same source, different behaviour. Code that silently overflowed at 32767 on the
Mega will not on the ESP32, and vice versa. `uint16_t` means 16 bits everywhere.
For a distance in mm topping out near 2000, `uint16_t` is exactly right and
documents the intent.

**Rule of thumb:** fixed-width types for anything touching hardware, registers,
or a wire protocol. Plain `int` is fine for a loop counter.

These exist in **both** C and C++ (via `<stdint.h>` / `<cstdint>`), so this one
carries over to ESP-IDF unchanged.

#### A2. Objects and methods — C++ only

```cpp
VL53L0X sensor;              // create an instance of type VL53L0X
sensor.init();               // call a method on it
sensor.setTimeout(500);
```

You already do this constantly — `Serial` is an object and `Serial.begin(115200)`
is a method call. `sensor` works identically; the only difference is that
`Serial` is pre-created by `Arduino.h` while you declare `sensor` yourself.

With two sensors you would write `VL53L0X sensorA; VL53L0X sensorB;` and each
carries its own internal state (I²C address, timeout, calibration). That is the
point of objects.

**C has no objects.** Appendix B shows the C equivalent: a `struct` holding the
state, plus functions that take a pointer to it as their first argument.

#### A3. `::` — the scope resolution operator — C++ only

```cpp
sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
```

`VL53L0X::VcselPeriodPreRange` reads as "the `VcselPeriodPreRange` constant that
lives inside `VL53L0X`". It is namespacing: `.` accesses a member of an
*instance*, `::` accesses something belonging to the *type itself*.

C has no `::` at all. C libraries fake it with name prefixes —
`VL53L0X_VCSEL_PERIOD_PRE_RANGE` — which is why C APIs have such long names.

#### A4. `struct` — grouping related values

```cpp
struct Range {
  bool     valid;
  uint16_t raw_mm;
  uint16_t filtered_mm;
};
```

A custom type bundling several fields. Used as:

```cpp
Range r = pollRange();      // get one back from a function
if (r.valid) {              // read a field with .
  Serial.println(r.filtered_mm);
}
```

Why: a function returns exactly one value, but `pollRange()` must communicate
three things — did we get a sample, what was it raw, what was it filtered.
Without a struct you would need globals or pointer out-parameters.

> **C/C++ difference you will hit reading examples online:** in C you must write
> `struct Range r = ...`, or `typedef struct {...} Range;` once so that a bare
> `Range r` works afterwards. In C++ the bare `Range r = ...` works with no
> typedef. If you copy C code full of `typedef struct`, that is the C idiom for
> the same thing — harmless, just redundant in C++.

#### A5. Arrays

```cpp
static uint16_t ring[5];     // five uint16_t slots, indices 0..4
ring[ringIdx++] = mm;        // write, then increment ringIdx
```

Identical in C and C++. Two things to know, because neither language protects
you:

- Indices start at **0**. `ring[5]` on a 5-element array is out of bounds.
- There is **no bounds checking**. `ring[7]` compiles fine and corrupts whatever
  memory sits there. This is the most common source of baffling embedded
  crashes, and why the code explicitly wraps with
  `if (ringIdx >= 5) ringIdx = 0;`

`ringIdx++` is post-increment: use the current value, *then* add one.
`++ringIdx` adds first. In array indexing the distinction matters.

#### A6. `volatile` and the ISR

```cpp
volatile bool sampleReady = false;
void IRAM_ATTR onRangeReady() { sampleReady = true; }
```

- **`volatile`** tells the compiler "this can change behind your back". Without
  it, the optimiser sees `while (!sampleReady)` in `loop()`, notices nothing in
  `loop()` writes it, caches the value in a register, and generates an infinite
  loop. `volatile` forces a fresh read from memory every time. Any variable
  shared between an ISR and normal code needs it. **Same in C and C++.**

- **`IRAM_ATTR`** is ESP32-specific, and a macro rather than a language keyword.
  It places the function in internal RAM instead of flash. ESP32 code normally
  executes from flash over SPI, but during a flash write — or while the other
  core does certain operations — flash is unavailable, and an ISR that tries to
  fetch instructions then will panic. On the Mega you would use `ISR()` macros
  instead; there is no AVR equivalent. **Also used in ESP-IDF**, same meaning.

#### A7. Function pointers

```cpp
attachInterrupt(digitalPinToInterrupt(PIN_INT), onRangeReady, FALLING);
```

Note `onRangeReady` with **no parentheses**. You are passing the function
*itself* as a value, not calling it. With `()` you would call it immediately and
pass its return value — a classic bug. You have likely already done this with
`attachInterrupt` on the Mega.

Same in C, and ESP-IDF leans on it much harder: callbacks, `xTaskCreate`, and
ISR registration all take function pointers.

#### A8. `static` at file scope

```cpp
static const int PIN_SDA = 21;
```

The valve driver used plain `const int MAC_VALVE_PIN = 28;` — functionally
identical inside a single file. `static` at file scope means "not visible to
other files", which matters once you split into multiple `.cpp`/`.c` files,
where it prevents name collisions. Hygiene, not meaning. Same in C.

---

### Part 2 — What changes when you move to ESP-IDF (C)

#### B1. No objects — pass a struct pointer instead

The universal C pattern, and the one Appendix B is built on:

```c
/* C++ */                        /* C equivalent */
VL53L0X sensor;                  vl53l0x_t sensor;
sensor.init();                   vl53l0x_init(&sensor);
sensor.setTimeout(500);          vl53l0x_set_timeout(&sensor, 500);
```

The struct holds what the object would have held; every function takes a pointer
to it as the first argument. `&sensor` means "the address of `sensor`", and
inside the function you reach fields with `->` instead of `.`:

```c
void vl53l0x_set_timeout(vl53l0x_t *dev, uint16_t ms) {
    dev->timeout_ms = ms;        /* -> because dev is a pointer */
}
```

**`.` vs `->` is purely about whether you hold the thing or its address.**
`sensor.timeout_ms` when you have the struct; `dev->timeout_ms` when you have a
pointer to it. (`dev->x` is shorthand for `(*dev).x`.)

#### B2. Pointers and out-parameters

C functions return one value, and ESP-IDF spends that return value on an error
code. So results come back through pointers:

```c
uint16_t distance;                               /* storage you own */
esp_err_t err = vl53l0x_read(&sensor, &distance); /* pass its address */
if (err == ESP_OK) {
    printf("%u mm\n", distance);                 /* now filled in */
}
```

`&distance` hands the function the address, and the function writes through it.
This is the single most common C idiom you will meet in ESP-IDF.

#### B3. `esp_err_t` — errors as return values

Almost every ESP-IDF function returns `esp_err_t`: `ESP_OK` (zero) on success,
or a nonzero code. There are no exceptions in C, so this is the only error
channel. Check it:

```c
esp_err_t err = i2c_master_transmit(dev, buf, len, timeout);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "write failed: %s", esp_err_to_name(err));
    return err;                                  /* propagate upward */
}
```

`esp_err_to_name()` turns the code into a readable string. `ESP_ERROR_CHECK(x)`
wraps a call and aborts the program on failure — fine during bring-up in
`app_main`, wrong anywhere a failure should be handled.

#### B4. `app_main()` instead of `setup()` / `loop()`

Arduino's `setup()` and `loop()` are not language features — they are a
convention the Arduino core provides, by calling them from its own `main()`.
ESP-IDF gives you one entry point and no loop:

```c
/* Arduino */                    /* ESP-IDF */
void setup() { ... }             void app_main(void) {
void loop()  { ... }                 /* setup work here */
                                     while (1) {
                                         /* loop body here */
                                         vTaskDelay(pdMS_TO_TICKS(20));
                                     }
                                 }
```

`app_main` runs as a FreeRTOS task. **Never busy-wait in it** — always yield with
`vTaskDelay()`, or the watchdog resets the chip. `pdMS_TO_TICKS(20)` converts
milliseconds to scheduler ticks.

#### B5. `ESP_LOGx` instead of `Serial.print`

```c
static const char *TAG = "vl53l0x";      /* per-file tag, by convention */

ESP_LOGI(TAG, "range %u mm", mm);        /* I = info  */
ESP_LOGW(TAG, "sample rejected");        /* W = warn  */
ESP_LOGE(TAG, "init failed");            /* E = error */
ESP_LOGD(TAG, "raw regs %02x %02x", a, b); /* D = debug, off by default */
```

Timestamped, tagged, colourised, and filterable per tag at runtime or build
time — strictly better than `Serial.print` for a multi-sensor system, because
you can see which subsystem each line came from.

The format specifiers are `printf`'s: `%u` unsigned, `%d` signed, `%f` float,
`%s` string, `%02x` two-digit hex, `%p` pointer. A mismatch between specifier
and argument type is a real bug class that C will not catch for you at runtime.
For the fixed-width types, strictly correct specifiers live in `<inttypes.h>`
(`PRIu16`, `PRIu32`), which you will see in ESP-IDF examples:

```c
ESP_LOGI(TAG, "range %" PRIu16 " mm", mm);
```

On ESP32 plain `%u` works for `uint16_t` and `uint32_t`; on some other targets
it does not, which is why the macros exist.

#### B6. The build system — CMake, not a sketch folder

ESP-IDF projects have a fixed shape, and files are **not** picked up
automatically the way `.ino` tabs are. Every source file must be listed:

```
project/
├── CMakeLists.txt           # project level
├── sdkconfig                # generated by menuconfig, do not hand-edit
├── main/
│   ├── CMakeLists.txt       # lists main.c and its dependencies
│   └── main.c
└── components/
    └── vl53l0x/
        ├── CMakeLists.txt   # lists vl53l0x.c, exposes include/
        ├── include/
        │   └── vl53l0x.h    # public header
        └── vl53l0x.c        # implementation
```

A **component** is ESP-IDF's unit of reuse — roughly an Arduino library. Drop a
directory under `components/` with a `CMakeLists.txt` and it builds and links
automatically; anything listing it under `REQUIRES` can `#include` its public
header.

#### B7. No auto-generated prototypes

In an `.ino` you can call `updateValve()` from `loop()` even though it is defined
40 lines below, because the IDE generated a prototype for you. In a real `.c` or
`.cpp` that is a compile error. You must either define functions before use, or
declare them first — in a header for anything shared, or at the top of the file
for file-local helpers.

A header also needs an include guard so it is not processed twice:

```c
#pragma once          /* modern, one line, works on every compiler here */
```

#### B8. Headers vs implementation

| File | Holds | Who reads it |
|---|---|---|
| `vl53l0x.h` | struct definitions, constants, function *declarations* | anything that uses the driver |
| `vl53l0x.c` | function *bodies*, file-local helpers and statics | nobody — it just compiles |

The header is the contract. Keep private registers and helpers out of it; if it
is not part of the interface, it belongs in the `.c`.

#### B9. The I²C driver changed in ESP-IDF v5.2

Worth knowing because most tutorials online are pre-v5.2 and the APIs are not
interchangeable:

| | Old (deprecated) | New (v5.2+) |
|---|---|---|
| Header | `driver/i2c.h` | `driver/i2c_master.h` |
| Setup | `i2c_param_config` + `i2c_driver_install` | `i2c_new_master_bus` + `i2c_master_bus_add_device` |
| Transfer | `i2c_master_cmd_begin` with a command link | `i2c_master_transmit_receive` |
| Model | one bus handle, address per transaction | a handle per device |

Appendix B uses the **new** API. If an example calls `i2c_master_cmd_begin`, it
is the old one — the concepts transfer but the function names do not.

---

### ESP32-vs-Mega portability notes

Coming from a Mega 2560, these will bite regardless of framework:

- **`Serial.printf()` does not exist on AVR.** It is an ESP32/ESP8266 core
  extension. On the Mega you need successive `Serial.print()` calls — which is
  exactly what the valve driver does.
- **`int` width changes**, per A1.
- **`float` vs `double`:** on AVR both are 32-bit (`double` is a lie). On ESP32,
  `double` is genuinely 64-bit and much slower, and the FPU is single-precision
  only. Stick with `float` and write `1.5f` rather than `1.5` in hot paths.
- **Pin numbers mean different things.** `GPIO 28` does not exist on the ESP32,
  and several GPIOs are strapping pins or wired to flash and unusable. Check a
  pinout before reusing pin constants.
- **The ESP32 has a watchdog.** A long blocking loop will trigger a reset —
  another reason the continuous-mode pattern avoids blocking.

---

### What to actually read

**If you stay on Arduino:** the [Arduino language reference](https://www.arduino.cc/reference/en/)
covers most of it, and the rest is items A1–A8 above.

**Since you are moving to ESP-IDF:** you need real C, not C++. Specifically
pointers, structs, and header/implementation separation — items B1–B9. The
[ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/)
is the reference, and the `examples/peripherals/i2c/` directory in your IDF
checkout is the most useful thing to read, because it is known-good code for
your exact framework version.

Note the inversion: for Arduino, C is the wrong thing to study because Arduino
is C++. For ESP-IDF, C is exactly the right thing to study. ST's own VL53L0X
HAL is C, so under ESP-IDF that code is now idiomatic for you rather than
something to translate.

---

## Appendix B: the ESP-IDF starter project

A working ESP-IDF project lives in
[`sprint 2/esp-idf/hopper-sensors/`](../sprint%202/esp-idf/hopper-sensors/).
It drives one VL53L0X and is structured so that adding the IMU and load cells
does not require touching `main.c`. See its `README.md` for build and wiring
instructions; this appendix explains the design.

It is written in **C**, per Appendix A.

### Requirement: ESP-IDF v5.2+

The project uses the `driver/i2c_master.h` API. On v5.1 or earlier it will not
compile. Most VL53L0X examples online use the older `driver/i2c.h` API.

### Layout

```
hopper-sensors/
├── CMakeLists.txt
├── main/
│   └── main.c                      app_main: bus setup, sensor table, loop
└── components/
    ├── sensor_common/
    │   └── include/sensor_iface.h  generic sensor interface (the vtable)
    ├── i2c_bus/                    one shared I2C bus + scanner
    └── vl53l0x/
        ├── vl53l0x.c               register-level driver
        ├── vl53l0x_iface.c         interface adapter + median filter
        └── include/
```

Three layers, each replaceable without disturbing the others:

| Layer | Knows about | Does not know about |
|---|---|---|
| `vl53l0x.c` | the chip's registers | the sensor interface |
| `vl53l0x_iface.c` | both | the application |
| `main.c` | the sensor interface | any specific sensor |

### The design problem

With several sensors, the naive `main.c` becomes a pile of special cases:

```c
vl53l0x_read(&tof, &mm);
mpu6050_read_accel(&imu, &ax, &ay, &az);
hx711_read(&cell, &counts);
/* ... each with its own error handling and units */
```

Every new sensor edits the control loop, and the loop accumulates knowledge of
every device. The fix is one interface that all drivers implement, so the loop
becomes:

```c
for (int i = 0; i < SENSOR_COUNT; i++) {
    sensor_poll(&g_sensors[i], &g_samples[i]);
}
```

Adding a sensor is then: write a driver, add one line to a table. The loop does
not change.

### The vtable pattern

C has no classes or virtual methods, so the standard substitute is a **vtable**:
a struct of function pointers that each driver fills in with its own functions.

```c
typedef struct {
    const char *type_name;
    esp_err_t (*init)(sensor_t *self);
    esp_err_t (*start)(sensor_t *self);
    esp_err_t (*poll)(sensor_t *self, sensor_sample_t *out);
    esp_err_t (*stop)(sensor_t *self);
} sensor_vtable_t;
```

Read `esp_err_t (*poll)(sensor_t *self, sensor_sample_t *out)` as: *a pointer,
named `poll`, to a function taking those two arguments and returning
`esp_err_t`*. The parentheses around `*poll` are required — without them,
`esp_err_t *poll(...)` declares a function returning a pointer, which is
something else entirely.

`self` is the explicit form of C++'s implicit `this`.

This is literally how C++ implements virtual methods; you are writing the
mechanism by hand instead of the compiler generating it.

A sensor instance then carries its vtable plus a type-erased pointer to its own
private state:

```c
struct sensor_s {
    const sensor_vtable_t *vtable;  /* shared, one per driver type */
    void                  *ctx;     /* per-instance private state  */
    const char            *label;
    bool     online;
    uint32_t sample_count;
    uint32_t error_count;
};
```

`void *` means "pointer to something unspecified" — C's escape hatch for
exactly this. The cost is that the compiler can no longer check the type: cast
`ctx` to the wrong struct and you get silent memory corruption, not an error.
Keeping each cast inside the driver that owns the `ctx` is what makes the
pattern safe in practice.

### Two independent notions of failure

`sensor_sample_t` carries a `valid` flag *and* `poll()` returns an `esp_err_t`.
That is not redundancy:

| | Meaning |
|---|---|
| `esp_err_t` | did the **transaction** work — did we talk to the chip? |
| `valid` | is the **number** trustworthy? |

The VL53L0X will happily return a confident-looking distance with a "sigma too
high" flag attached. The I²C read succeeded; the datum is garbage. Collapsing
these two into one value means either discarding good diagnostics or treating
bad data as good.

`ESP_ERR_NOT_FINISHED` is a third case: no new sample yet. Normal, expected most
iterations, and not an error — the health counters deliberately ignore it.

### Non-blocking by construction

`poll()` must not block. If no sample is ready it returns immediately. This is
what stops one slow sensor from stalling the control loop, and it is why the
driver exposes continuous mode rather than only single-shot ranging.

In `app_main` the loop then looks like:

```c
while (1) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensor_sample_t s;
        if (sensor_poll(&g_sensors[i], &s) == ESP_OK && s.valid) {
            g_samples[i] = s;
        }
    }
    /* ... GNC update reads g_samples[] ... */
    vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
}
```

That `vTaskDelay` is mandatory, not politeness. Without it the idle task never
runs and the task watchdog reboots the chip — the ESP-IDF equivalent of the
blocking-`delay()` habit being actively harmful.

### No dynamic allocation

`vl53l0x_sensor_create()` allocates nothing; the caller owns both structs,
typically as file-scope statics in `main.c`. On a microcontroller this is the
right default: no `malloc` means no fragmentation, no leaks, and a memory
figure fixed at link time.

### Units converted at the boundary

The chip speaks millimetres; `sensor_iface` speaks metres, and every sample is
tagged with its physical quantity (`SENSOR_QTY_DISTANCE_M`). Converting once, in
the adapter, means nothing downstream has to remember which unit it holds.

### Filtering policy lives in the adapter

The median filter is in `vl53l0x_iface.c`, not the driver. The driver reports
what the chip said; the adapter decides what to do about it. That keeps the
policy in one place and leaves the driver reusable if you want raw samples.

Why a median rather than an average, and what it costs, is in section 11 —
briefly: VL53L0X failures are isolated gross outliers, so a median rejects them
while an average smears them, and the price is roughly `taps/2` samples of
group delay.

### Where the GNC code goes

`app_main` marks the spot, with the two corrections from section 11 noted
inline: tilt (`altitude ≈ range × cos θ`, needs the IMU) and latency (true
measurement epoch is ~`budget/2` before the timestamp, plus filter delay).

Each sample carries its own `timestamp_us`, so a loop can reject stale data
rather than assuming freshness.

### Verification status

The code compiles clean under `-Wall -Wextra -Wshadow -Wsign-compare` at both
gnu99 and gnu17, and its logic was tested on a host against a simulated
VL53L0X register file — 48 checks covering init, timing-budget arithmetic and
clamping, VCSEL period validation, offset application including the
negative-clamp path, sanity-window and status rejection, continuous-mode reads,
health counters, null guards, and offset calibration. The median filter was
checked against an independent reference over 4000 samples with injected
outliers (exact agreement), steady-state lag matching the predicted
`(taps/2) × step`. All passes under ASan and UBSan.

Two bugs surfaced that way and are fixed: a missing `<stddef.h>` for `NULL` in
`sensor_iface.h`, and `%lld` where `PRId64` was needed for `int64_t`.

**None of it has run on hardware.** Host tests cannot validate the undocumented
register sequences, real I²C timing, or the analog front end. Expect to debug on
first flash, and start with the bus scan — if `0x29` does not appear, the
problem is electrical, not software.

---

## Appendix C: Arduino Nano ESP32 notes

The Arduino Nano ESP32 is an **ESP32-S3**, in a u-blox NORA-W106 module. It runs
the project in Appendix B, but three things differ from a classic ESP32 DevKit
and all three will stop you if unaddressed.

### 1. GPIO 22 does not exist on the ESP32-S3

This is the one that bites hardest, because the failure looks like a wiring
problem.

The ESP32-S3 has **GPIO 0–21 and GPIO 26–48**. The numbering skips 22, 23, 24
and 25 entirely — they are not pins that are hard to reach, they are absent
from the silicon. So the classic ESP32 default of `SDA = 21, SCL = 22` cannot
work: `gpio_config()` rejects GPIO 22 and the bus never comes up.

Also avoid **GPIO 26–32** on the S3: they are wired to the module's SPI flash
and PSRAM. Using them gets you a board that misbehaves or will not boot.

`main.c` now carries a `_Static_assert` for each pin, so a bad choice is a
build error with a readable message rather than a debugging session.

### 2. Arduino pin labels are not GPIO numbers

In the Arduino IDE, `A4` is a *label*. The core holds a lookup table and
remaps it to a real GPIO before touching the hardware. ESP-IDF has no core and
does no remapping — it wants the raw number from the schematic.

For the Nano ESP32 the mapping is:

| Silkscreen | Raw GPIO | |
|---|---|---|
| `A4` | **11** | SDA |
| `A5` | **12** | SCL |
| `A0`–`A3` | 1, 2, 3, 4 | |
| `A6`, `A7` | 13, 14 | |
| `D2`–`D7` | 5, 6, 7, 8, 9, 10 | |
| `D8`, `D9` | 17, 18 | |
| `D10` | 21 | SS |
| `D11`, `D12`, `D13` | 38, 47, 48 | MOSI, MISO, SCK |
| `D0`, `D1` | 44, 43 | RX, TX |

So: wire the sensor to the pins **labelled** `A4` and `A5`, and tell ESP-IDF
**11** and **12**.

> The authoritative source is `variants/arduino_nano_nora/pins_arduino.h` in
> the `arduino-esp32` core. Note that file defines the mapping *twice*, under
> `#if BOARD_HAS_PIN_REMAP` — the Arduino-facing branch where `A4 = 21` is the
> *label* number, and the raw branch where `A4 = 11` is the real GPIO. Reading
> the wrong branch is an easy way to get a plausible, wrong answer.

### 3. Native USB, so the console needs configuring

The Nano ESP32 has **no USB-to-UART bridge chip**. It uses the ESP32-S3's
native USB peripheral, which means:

- It enumerates as **`/dev/ttyACM0`**, not `/dev/ttyUSB0`.
- `idf.py monitor` will connect and show **nothing at all** until you point the
  console at USB: `idf.py menuconfig` → *Component config* → *ESP System
  Settings* → *Channel for console output* → **USB Serial/JTAG Controller**.
  By default the logs go to the physical UART0 pins, where nothing is
  listening.
- The port **disappears on every reset**, because the USB device is implemented
  by the chip being reset. If flashing fails, double-tap the reset button to
  drop into the ROM bootloader and retry.

On WSL, pass the device through with `usbipd` before `/dev/ttyACM0` appears at
all.

### Build

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

`set-target` regenerates `sdkconfig`, so set the console option *after* it, not
before.

### The bootloader trade

The board ships with an Arduino bootloader. Flashing with `idf.py` replaces it.
That is expected and reversible, but to use the Arduino IDE with this board
again you will need to restore it — double-tap reset, then *Tools → Burn
Bootloader*, or re-flash any sketch over DFU. Worth knowing before you flash.

### What does not change

Everything else in the project is portable as written:

- The driver is pure register access over the ESP-IDF I²C API — no
  chip-specific code.
- `i2c_master.h`, `esp_timer`, FreeRTOS and `ESP_LOGx` are all identical
  across variants.
- 3.3 V GPIO, same as any ESP32. Power the breakout from `3V3`, not `VBUS`.
- The VL53L0X itself is unchanged: same `0x29`, same 400 kHz, same everything
  in sections 1–11.

### If you would rather stay in the Arduino IDE

The Nano ESP32 is fully supported by the Arduino IDE, and the C++ listings in
[section 7](#7-code-minimal-bring-up) and
[section 8](#8-code-continuous-mode-with-interrupt-and-filtering) run on it
with one change — pass the pins explicitly, using the **labels**, which the
core then remaps:

```cpp
Wire.begin(A4, A5);     // not Wire.begin(21, 22)
```

That is a legitimate way to get first light on the sensor quickly, and it
sidesteps all three issues above. The ESP-IDF project is the better home for
flight software, but there is no reason the first bring-up has to wait for it.
