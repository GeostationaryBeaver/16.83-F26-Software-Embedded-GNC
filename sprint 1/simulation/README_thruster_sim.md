# thruster_sim.py

Lumped-parameter simulation of a cold gas thruster with PWM valve throttling.
Models tank -> valve orifice -> chamber (plenum) -> choked nozzle -> ambient,
and produces a thrust-vs-time profile over a 15 s demo (5 s closed, 5 s PWM'd,
5 s full open).

## Running

Requires `numpy` and `matplotlib` (project uses the `spacesys` conda env):

```
conda activate spacesys
python thruster_sim.py
```

Prints per-phase mean/peak thrust and mean mass flow. The simulation itself
runs instantly; the plot window then draws each curve incrementally (sweeping
left to right) rather than appearing all at once, controlled by
`ANIMATION_FRAMES` / `ANIMATION_PAUSE_S` near the top of the file. Axis limits
are fixed to the final data extents before the sweep starts, so the animation
is purely visual — once the sweep finishes, `thruster_profile.png` is saved
and is identical to a static one-shot plot of the complete data.

## Parameters

- **USER PARAMETERS**: tank pressure, gas properties, valve/nozzle geometry,
  PWM frequency/duty cycle, and the integration timestep. Edit these to match
  the actual test article.
- **LOAD CELL DATA ACQUISITION**: constants pulled directly from
  `loadcells.ino` (serial baud, HX711 gain, samples averaged per reading,
  read timeout) so the simulated thrust trace can be compared against the
  real test-stand logging cadence.
- Two constants in that section are marked **HARDWARE-DEPENDENT — ASK
  STRUCTURES SUBTEAM**: the HX711 ADC sample rate (set by a physical pin
  strap on the load-cell amplifier board, not by the firmware) and the load
  cell's rated full-scale capacity. Confirm these with the structures
  subteam before trusting them.

## Output

- Top plot: full 15 s thrust profile (N), phase-labeled, with a 50 ms moving
  average overlaid.
- Bottom plots: zoomed-in view of a few PWM cycles — thrust (N) and valve
  command vs. actual valve position (first-order lag).

Chamber pressure is no longer plotted; it is still computed internally (mass
balance) because it drives the nozzle choking condition. Tank pressure is
reported in the console output and plot title in psi; thrust stays in
Newtons throughout.
