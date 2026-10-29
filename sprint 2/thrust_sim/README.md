# Ball-valve thruster simulation (`thrust_sim.py`)

Pure simulation (no telemetry, no animation) of a cold-gas thruster throttled by a
ball valve, fed from a blowdown tank through a pressure regulator:

```
tank (blowing down) --[regulator]--> feed --[ball valve]--> plenum --[nozzle]--> ambient
```

Three equal-length phases are run back to back:

| phase | duration            | valve angle                  | result          |
|-------|---------------------|-------------------------------|-----------------|
| 1     | `PHASE_DURATION_S`  | 0 deg (closed)                | no thrust       |
| 2     | `PHASE_DURATION_S`  | `VALVE_THROTTLE_ANGLE_DEG`    | throttled thrust|
| 3     | `PHASE_DURATION_S`  | 90 deg (full open)            | full thrust     |

## How to run

```
conda activate spacesys
cd "16.83-F26-Software-Embedded-GNC/sprint 2/thrust_sim"
python thrust_sim.py
```

Requires only `numpy` and `matplotlib` (already in the `spacesys` env). The script
prints per-phase thrust/mass-flow stats and the regulator dropout time (if any) to
the console, saves two PNGs next to the script (`thruster_profile.png` and, if
`SHOW_VALVE_CHARACTERISTIC` is on, `valve_characteristic.png`), and opens the plot
windows (`plt.show()`).

## What the plots show

**`thruster_profile.png`** (one figure, four panels):
- **Thrust vs. time** — instantaneous thrust plus a 50 ms moving average, with the
  three phases shaded and labeled.
- **Pressures vs. time** — tank, regulator outlet, and chamber (plenum) pressure, all
  in psig. The regulator outlet is flat at its setpoint until the tank pressure falls
  through it (the "dropout" knee), after which the outlet tracks the tank.
- **Valve angle vs. time** — commanded angle (dashed) vs. actual angle (the actuator
  has a first-order lag, so actual angle ramps toward the command rather than
  stepping instantly).
- **Mass flow vs. time** — mass flow into the plenum through the valve vs. mass flow
  out through the nozzle throat (grams/second).

**`valve_characteristic.png`** (two panels, independent of the phase schedule above):
- **Flow-area fraction vs. angle** — the ball valve's effective open area (normalized
  to its fully-open area) as a function of rotation angle. This is an S-curve, not a
  straight line: with the default geometry, a half-open valve (45 deg) passes only
  about 5% of the full-open area, not 50%, because the open area is the lens-shaped
  overlap of two circular bores as the ball rotates, and most of that overlap only
  appears in roughly the last 30 degrees of rotation.
- **Steady thrust vs. angle** — the thrust the thruster would settle to at each angle
  if the feed pressure were held at the regulator setpoint indefinitely (i.e. before
  any tank dropout). Useful for picking a `VALVE_THROTTLE_ANGLE_DEG` that gives a
  specific target thrust.

## User parameters

All of these live in the `USER PARAMETERS` block at the top of `thrust_sim.py`.
Everything below that block (`DERIVED CONSTANTS` onward) is computed from these and
should not normally need editing.

### Supply: tank + regulator

| variable | units | meaning |
|---|---|---|
| `TANK_INITIAL_PRESSURE_PSIG` | psig | Tank pressure at the start of the run (gauge, i.e. above atmospheric). Higher = more total impulse and a later (or no) regulator dropout. |
| `TANK_VOLUME_M3` | m^3 | Physical volume of the tank. Together with the mass flow drawn, this sets how fast tank pressure falls and therefore when (or whether) the regulator drops out during the run. Smaller tank -> dropout happens sooner. |
| `REGULATOR_SETPOINT_PSIG` | psig | The regulated feed pressure delivered to the ball valve, as long as the tank can support it. This is effectively your "supply pressure" for sizing thrust, independent of the raw tank pressure. |
| `AMBIENT_PRESSURE_PA` | Pa | Ambient (back) pressure the nozzle exhausts into. 101325 Pa = sea level. |

### Gas properties (default: nitrogen)

| variable | units | meaning |
|---|---|---|
| `GAMMA` | - | Ratio of specific heats (cp/cv) of the working gas. |
| `MOLAR_MASS` | kg/mol | Molar mass of the gas; sets the specific gas constant `R_GAS = R_UNIV / MOLAR_MASS`. |
| `GAS_TEMP_K` | K | Gas temperature, assumed constant everywhere (isothermal tank blowdown and isothermal plenum fill). |

### Ball valve

| variable | units | meaning |
|---|---|---|
| `BALL_BORE_DIAMETER_M` | m | Diameter of the through-bore in the ball (and of the matching seat port). This sets the fully-open flow area. **Keep this at or above the nozzle throat/inlet diameter** — if the valve bore is much smaller than the nozzle, the valve becomes the bottleneck instead of the nozzle, and full-open thrust will be far below what the nozzle is sized for. |
| `BALL_DIAMETER_M` | m | Outer diameter of the ball itself. Larger ball (relative to the bore) makes the opening curve sharper — area opens more slowly near the closed end and more quickly near full open. **Must be at least `2 * BALL_BORE_DIAMETER_M`** for the valve to fully seal at 0 deg; the default (10 mm ball, 4.02 mm bore) satisfies this with margin. |
| `VALVE_CD` | - | Discharge coefficient of the valve orifice (0-1); accounts for real flow losses vs. the ideal orifice equation. |
| `VALVE_TAU_S` | s | First-order lag time constant of the valve actuator: how quickly the actual angle catches up to a step change in commanded angle. Larger = sluggish actuator, visible as a ramp instead of a step in the "valve angle vs. time" plot. |
| `VALVE_THROTTLE_ANGLE_DEG` | deg | The angle held during phase 2 (0 = closed, 90 = full open). Use the "steady thrust vs. angle" plot to pick a value that targets a specific thrust level. |

### Nozzle geometry

Values below are the structures subteam's numbers from the "6.06N Nozzle
Calculations" sizing (same geometry used in `term_project/thruster_sim.py` and
`sprint 1/Claude generated thrust model/nozzle_thrust.py`).

| variable | units | meaning |
|---|---|---|
| `THROAT_DIAMETER_M` | m | Nozzle throat diameter. Sets the choked mass flow rate for a given chamber pressure. |
| `EXIT_DIAMETER_M` | m | Nozzle exit diameter. Together with the throat diameter this sets `EXPANSION_RATIO = (exit/throat)^2`, which determines exit Mach number, exit pressure, and exit velocity. |
| `INLET_DIAMETER_M` | m | Nozzle inlet diameter. Kept here for reference/documentation; not used directly in the flow equations below (the plenum connects to the throat, not the inlet bore). |
| `NOZZLE_CD` | - | Nozzle discharge coefficient (0-1), accounting for real losses vs. the ideal isentropic nozzle equations. |
| `CHAMBER_VOLUME_M3` | m^3 | Volume of the plenum between the valve and the nozzle throat. Smaller volume responds faster to valve changes (less "accumulator" effect) but makes the simulation stiffer (may need a smaller `DT`). |

### Run / numerics

| variable | units | meaning |
|---|---|---|
| `PHASE_DURATION_S` | s | Length of each of the three phases (closed / throttled / full open). Total run length is `3 * PHASE_DURATION_S`. |
| `DT` | s | Fixed integration time step (explicit forward-Euler). Decrease if you see numerical ringing or instability in chamber/tank pressure, especially after changing `CHAMBER_VOLUME_M3` or `BALL_BORE_DIAMETER_M` to something much smaller. |
| `SHOW_VALVE_CHARACTERISTIC` | bool | If `True`, also compute and plot the steady thrust/area-fraction vs. angle curves (`valve_characteristic.png`). Set `False` to skip that extra figure and speed up the run slightly. |

## Notes on the model

- Pressures suffixed `_PSIG` are **gauge** pressures (relative to ambient); everything
  internal to the physics is converted to and computed in **absolute** Pascals.
- The regulator is modeled as ideal: it holds `REGULATOR_SETPOINT_PA` exactly as long
  as the tank pressure is above it, then "drops out" and the feed pressure simply
  equals the (falling) tank pressure. There is no droop or response lag modeled.
- The chamber (plenum) pressure is clamped to never exceed the feed pressure — it
  physically cannot, and this also prevents forward-Euler overshoot when the valve
  snaps wide open.
- The ball valve's effective area uses the lens-overlap (circle-circle intersection)
  formula, which is exact at both ends (0 at fully closed, the full bore area at fully
  open) and reproduces the nonlinear "most of the flow authority is in the last 30
  degrees" behavior of a real ball valve.
- The nozzle/valve orifice flow equations (`orifice_mdot`) and the isentropic nozzle
  exit relations (`exit_pressure_ratio`) are unchanged from the earlier sprint 1 PWM
  version of this simulation — only the throttling element (ball valve instead of a
  PWM'd on/off orifice) and the supply model (finite blowdown tank + regulator instead
  of an infinite constant-pressure tank) are new.
