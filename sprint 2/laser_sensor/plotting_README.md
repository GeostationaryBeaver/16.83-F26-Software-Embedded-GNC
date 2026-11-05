# Laser logging and plotting

Quick guide to recording and plotting VL53L0X range data. Same workflow as the
load cells: a sketch that streams CSV, a logger that captures it, a plotter
that draws it.

If you just want to confirm the sensor works at all, use
`arduino-bringup/` instead — that prints human-readable output and does not
record anything.

## Files

| File | What it does |
|---|---|
| `laser_logging/laser_logging.ino` | Streams `millis,raw_mm,range_mm,status` continuously; accepts commands at any time |
| `LaserLogging.py` | Captures a trial to CSV. Needs the COM port, so run with `python.exe` on WSL |
| `Laser_plotting.py` | Plots the CSVs. Needs matplotlib, so run with `python3` |

## 1. Upload the sketch

- Board: *Tools → Board → Arduino ESP32 Boards → Arduino Nano ESP32*
  (install "Arduino ESP32 Boards" by Arduino in Boards Manager first — without
  it the IDE compiles for AVR and `Wire.begin()` rejects pin arguments)
- Library: "VL53L0X" by Pololu
- Wiring: `VIN`→`3V3`, `GND`→`GND`, `SDA`→`A4`, `SCL`→`A5`

You can check the Serial Monitor to see rows appearing, but **close it before
running the logger** — they cannot hold the port at the same time.

## 2. Record a trial

```bash
python.exe LaserLogging.py --duration 60 --out hover1
```

With offset calibration (recommended — a 10–20 mm raw offset is normal):

```bash
python.exe LaserLogging.py --zero 100 --duration 60 --out hover1
```

That prompts you to place a flat target exactly 100 mm away, computes the
offset from 50 samples, then prompts again before logging starts.

Useful flags:

| Flag | Effect |
|---|---|
| `--port COM7` | if the board is not on COM6 |
| `--profile 1` | 0 default, 1 high speed (50 Hz), 2 long range, 3 high accuracy |
| `--zero 100` | calibrate against a target at 100 mm |
| `--duration 60` | trial length in seconds |
| `--list-ports` | show detected ports and exit |
| `--yes` | do not wait for Enter at the prompts |
| `--quiet` | do not echo every row |

`--list-ports` is worth knowing: the Nano ESP32 re-enumerates on every reset,
so its COM number can change between runs.

## 3. Plot it

```bash
python3 Laser_plotting.py hover1.csv
python3 Laser_plotting.py hover1.csv hover2.csv --save trials.png
python3 Laser_plotting.py hover1.csv --smooth 5 --rate
python3 Laser_plotting.py hover1.csv --target 250
```

Use `python3` here, not `python.exe` — matplotlib lives in the WSL conda env
while the logger needs Windows Python for the COM port. Same split as the load
cell scripts.

| Flag | Effect |
|---|---|
| `--target 250` | draw a target distance and report error against it |
| `--no-zero-base` | let the y axis auto-scale instead of starting at 0 mm |
| `--column raw_mm` | plot pre-offset values instead of corrected |
| `--column status` | plot the status code, to see *why* samples failed |
| `--smooth 5` | 5-sample rolling median, with the raw trace faded underneath |
| `--rate` | add a panel showing sample-to-sample interval |
| `--save out.png` | write a PNG instead of opening a window |
| `--no-dropouts` | hide the rejected-sample marks |
| `--no-stats` | hide the caption |

`python3 Laser_plotting.py -h` for the rest.

## The y axis starts at 0 mm

Distance has a true zero, so by default the range axis runs from 0 rather than
auto-scaling to the data. That keeps the height of a trace proportional to the
distance it represents, and makes the mm and metres axes line up readably
(200 mm ↔ 0.2 m, 400 mm ↔ 0.4 m).

Auto-scaling instead picks an arbitrary floor, which exaggerates small
movements: a hover holding 265 ±3 mm looks violently unstable on an axis
spanning 258 to 270.

The trade-off is that once the axis runs 0 to 500, a 2 mm noise band is
invisible. When you are looking at *noise* rather than absolute distance:

```bash
python3 Laser_plotting.py hover1.csv --no-zero-base
```

This applies to `range_mm` and `raw_mm` only. `--column status` is left
auto-scaled, since a status code of 0 has no magnitude to be proportional to.

## Measuring against a target

`--target 250` draws a dashed reference line at 250 mm and adds two caption
lines per trace:

```
vs 250 mm target: mean error +10.00 mm   sd 2.00 mm   rms 10.20 mm
worst 12.00 mm off   0% within +/-5 mm   n=100
```

**`mean error` is signed, and the sign is the useful part.** Positive means the
sensor reads consistently long, negative consistently short. It is also the
offset correction you need, negated: a mean error of +10 mm means subtracting
10 mm would centre the readings. In practice you would re-run
`LaserLogging.py --zero <mm>` rather than doing that arithmetic yourself.

**`sd` is the spread of the error, which equals the spread of the values.**
Subtracting a constant target shifts a distribution without changing its
width, so this number is identical to the `noise sd` on the line above. It is
reported because it is the right measure of *repeatability* — but on its own it
says nothing about whether the sensor is near the target. A sensor 10 cm off
with perfect consistency has an excellent sd.

**`rms` is the single number that answers "how far off am I".** It combines
bias and scatter — `rms² = mean² + sd²` — so unlike sd it cannot be made to
look good by being consistently wrong. If you want one figure of merit, use
this one.

`worst` is the largest single deviation, and `within ±5 mm` is the fraction of
samples inside a 5 mm band, which is often more informative than any average
when you care about worst-case behaviour.

Dropouts are excluded from all of these — a rejected sample has no distance to
compare. `n=` reports how many samples actually contributed, so it will be
lower than the total when the trial had dropouts.

`--target` requires `--column range_mm` or `raw_mm`; it is refused on a status
or timestamp axis, where a distance line would be meaningless.

## Reading the plot

**Red ticks along the bottom** are rejected samples. The line *breaks* at each
one rather than drawing through it — a gap means data is genuinely missing,
not that the sensor measured a straight line.

**The caption** reports, per trace:

- `steady` — the distance it was mostly holding, with noise standard deviation
- `span` — min to max
- `Hz` and `ms` — actual sample cadence, which should match your profile
  (33 ms default, 20 ms high speed, 200 ms high accuracy)
- dropout count and percentage

**The last caption line** names the status codes behind the rejections, which
is the useful diagnostic:

| Status | Means | Usually |
|---|---|---|
| `signal too low` | not enough light came back | target too far, too dark, or too angled |
| `sigma too high` | noise too high to trust the peak | bright ambient light, or a weak return |
| `phase/wraparound` | ambiguous distance | target beyond range |
| `below min range` | closer than measurable | target under ~30 mm |
| `hardware fail` | chip fault | wiring or a bad module |

## If something looks wrong

**Every row is `nan`.** The sensor saw no valid target. Check the lens is
unobstructed and aim at a wall 10–50 cm away. Plot `--column raw_mm` to see
what it was reading before validation rejected it.

**Lots of dropouts.** Check the target is inside 30–1600 mm, and that nothing
is inside the 25° cone — at 1 m that cone is ~44 cm across, and the sensor
reports the *nearest* strong return, not what you aimed at.

**Readings implausibly short.** Something in the cone you did not intend, or a
smudge on the lens acting as cover glass.

**Cadence much slower than the profile.** The sensor converges early on bright
close targets and burns the whole budget on dark far ones, so the interval is
data-dependent up to the budget ceiling.

## Verification status

The sketch compiles clean under `g++ -Wall -Wextra` against stub Arduino
headers. The logger was tested against a simulated `laser_logging.ino` over a
fake serial port (24 checks: CSV structure, metadata, calibration protocol,
profile selection, overwrite guard, port diagnostics, argument validation).
The plotter was tested against synthetic CSVs with known parameters (33 checks:
statistics recovered the fixture values, dropout handling, segment breaking,
median smoothing, metadata parsing, edge cases) and every figure was rendered
and inspected. The `--target` error statistics were verified against a
hand-computed fixture (21 further checks: mean, sd, RMS and worst-case all
exact to nine decimals, the `rms² = mean² + sd²` identity, sign convention,
dropout exclusion, boundary behaviour). Zero-basing was verified across 13
combinations (both columns, the opt-out, targets above and below the data,
smoothing, the cadence panel, and the status axis exclusion). Its palette passes all six `validate_palette.js` checks in both
light and dark modes.

**None of it has run against the real sensor** — that is what it is for.
