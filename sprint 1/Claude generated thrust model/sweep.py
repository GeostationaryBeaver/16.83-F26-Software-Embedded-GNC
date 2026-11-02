"""
Sweep pulse duty cycle or frequency and compare the delivered thrust.

    python sweep.py --duty 0.1 0.25 0.5 0.75 1.0
    python sweep.py --freq 5 10 20 50 100
    python sweep.py --duty 0.25 0.5 --freq 10 50 --out grid.png

The point of the sweep: pulsed mean thrust is NOT duty_cycle * steady_thrust.
The plenum stays partly charged between pulses, so the mean always exceeds the
naive product. The "ideal" reference line on the chart is that product, and the
gap to the curve is the effect being measured.

Read the gain alongside the propellant column, not on its own. Raising the
frequency does not buy thrust for free: at 200 Hz the valve barely closes
between pulses, so flow climbs from 5.3 to 9.0 g/s and Isp slips slightly. The
honest summary is that high-frequency pulsing approaches steady operation in
both thrust AND consumption -- it is not a more efficient way to throttle.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from nozzle_thrust import Model, Phase, Schedule, simulate
from plot_thrust import (
    COLOR_COMMAND,
    COLOR_GRID,
    COLOR_TEXT,
    COLOR_TEXT_MUTED,
    COLOR_TEXT_SECONDARY,
    COLOR_THRUST,
    _RC,
)

# Categorical slots 1-3 from the dataviz reference palette, in fixed order.
# All-pairs forms cap at three series, which is why --freq/--duty take at most
# three values when both are given.
SERIES_COLORS = (COLOR_THRUST, COLOR_COMMAND, "#1baf7a")


def steady_mean(model: Model, duration: float = 1.0) -> float:
    """Mean thrust with the valve held open, including its pressure drop."""
    r = simulate(model, Schedule([Phase("open", duration)]), sample_dt=200e-6)
    return r.total_impulse / duration


def pulse_point(
    model: Model, frequency: float, duty: float, duration: float = 1.0
) -> dict:
    """Mean thrust and propellant for one (frequency, duty) pair."""
    if duty <= 0.0:
        return {"frequency": frequency, "duty": duty, "mean_thrust": 0.0,
                "propellant": 0.0, "peak_thrust": 0.0}
    schedule = Schedule([
        Phase("pulse", duration, frequency=frequency, duty_cycle=duty)
    ])
    r = simulate(model, schedule, sample_dt=100e-6)
    return {
        "frequency": frequency,
        "duty": duty,
        "mean_thrust": r.total_impulse / duration,
        "propellant": r.propellant_used,
        "peak_thrust": float(r.thrust.max()),
    }


def _format_table(rows: list[dict], steady: float) -> str:
    out = [
        "Freq (Hz)   Duty      Mean F (N)   Ideal F (N)   Gain vs ideal   Peak F (N)   N2 (g/s)",
        "-" * 94,
    ]
    for r in rows:
        ideal = steady * r["duty"]
        gain = (r["mean_thrust"] / ideal - 1.0) * 100.0 if ideal > 1e-9 else float("nan")
        gain_s = "      -    " if ideal <= 1e-9 else "%+10.1f%%" % gain
        out.append(
            "%9.1f %6.0f%% %14.3f %13.3f %s %12.3f %10.3f"
            % (r["frequency"], r["duty"] * 100, r["mean_thrust"], ideal,
               gain_s, r["peak_thrust"], r["propellant"] * 1000.0)
        )
    out.append("-" * 94)
    out.append("Steady (valve fully open): %.3f N" % steady)
    return "\n".join(out)


def plot_sweep(
    rows: list[dict],
    steady: float,
    x_key: str,
    series_key: str,
    path: Path,
    psi: float = 200.0,
) -> Path:
    """Mean thrust against the swept variable, one line per series value."""
    x_label = "Duty cycle" if x_key == "duty" else "Pulse frequency  (Hz)"
    fig, ax = plt.subplots(figsize=(9.5, 5.6))

    series_values = sorted({r[series_key] for r in rows})
    log_x = x_key == "frequency" and len({r["frequency"] for r in rows}) > 2

    for i, sv in enumerate(series_values):
        pts = sorted((r for r in rows if r[series_key] == sv), key=lambda r: r[x_key])
        xs = [r[x_key] for r in pts]
        ys = [r["mean_thrust"] for r in pts]
        color = SERIES_COLORS[i % len(SERIES_COLORS)]
        label = (
            "%g%% duty" % (sv * 100) if series_key == "duty" else "%g Hz" % sv
        )
        ax.plot(xs, ys, color=color, linewidth=2.0, marker="o",
                markersize=5.5, zorder=3, label=label)

    # Reference: naive duty * steady. The gap to the curve is the plenum effect.
    if x_key == "duty":
        duties = np.linspace(0, 1, 50)
        ax.plot(duties, steady * duties, color=COLOR_TEXT_MUTED, linewidth=1.3,
                linestyle=(0, (5, 3)), zorder=2)
        ax.annotate(
            "ideal: duty x steady", xy=(0.62, steady * 0.62),
            xytext=(0, -16), textcoords="offset points", ha="center",
            fontsize=9, color=COLOR_TEXT_MUTED,
        )
    elif len(series_values) == 1:
        # Frequency sweep at one duty: the ideal is a constant, so label it.
        ideal = steady * series_values[0]
        ax.axhline(ideal, color=COLOR_TEXT_MUTED, linewidth=1.3,
                   linestyle=(0, (5, 3)), zorder=2)
        ax.annotate(
            "ideal: duty x steady = %.2f N" % ideal,
            xy=(0.012, ideal), xycoords=("axes fraction", "data"),
            xytext=(0, -15), textcoords="offset points",
            fontsize=9, color=COLOR_TEXT_MUTED,
        )
    ax.axhline(steady, color=COLOR_TEXT_MUTED, linewidth=1.0,
               linestyle=(0, (2, 2)), zorder=2)
    # Anchor in axes fraction: x=0 is off-scale on the log frequency axis.
    ax.annotate(
        "steady, valve open: %.2f N" % steady,
        xy=(0.012, steady), xycoords=("axes fraction", "data"),
        xytext=(0, 6), textcoords="offset points",
        fontsize=9, color=COLOR_TEXT_MUTED,
    )

    if log_x:
        ax.set_xscale("log")
        ax.set_xticks([r["frequency"] for r in rows])
        ax.get_xaxis().set_major_formatter(
            matplotlib.ticker.FuncFormatter(lambda v, _: "%g" % v)
        )
    ax.set_xlabel(x_label, labelpad=6)
    ax.set_ylabel("Mean thrust  (N)", labelpad=8)
    ax.set_ylim(0.0, steady * 1.18)
    if x_key == "duty":
        ax.set_xlim(0.0, 1.05)
    if len(series_values) > 1:
        ax.legend(frameon=False, loc="lower right", fontsize=9,
                  bbox_to_anchor=(0.99, 0.03))

    ax.set_title(
        "Pulsed mean thrust vs %s" % ("duty cycle" if x_key == "duty" else "frequency"),
        loc="left", fontsize=13.5, fontweight="semibold", color=COLOR_TEXT, pad=26,
    )
    # Name the fixed variable, so a single-series chart needs no legend.
    fixed = ""
    if len(series_values) == 1:
        sv = series_values[0]
        fixed = (
            "at %g%% duty cycle" % (sv * 100) if series_key == "duty"
            else "at %g Hz" % sv
        )
    ax.annotate(
        ("%s   |   " % fixed if fixed else "")
        + "%g psig N2, sea level   |   dashed line is duty x steady" % psi,
        xy=(0.0, 1.0), xycoords="axes fraction", xytext=(0, 9),
        textcoords="offset points", ha="left", va="bottom",
        fontsize=9.5, color=COLOR_TEXT_SECONDARY,
    )

    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--duty", type=float, nargs="+",
                    help="duty cycles to sweep, 0..1")
    ap.add_argument("--freq", type=float, nargs="+",
                    help="pulse frequencies in Hz")
    ap.add_argument("--psi", type=float, default=None,
                    help="supply gauge pressure, psi (default 200)")
    ap.add_argument("--duration", type=float, default=1.0,
                    help="simulated seconds per point (default 1.0)")
    ap.add_argument("--out", type=Path, default=Path("sweep.png"))
    args = ap.parse_args()

    duties = args.duty or [0.1, 0.25, 0.5, 0.75, 1.0]
    freqs = args.freq or [20.0]

    for name, vals in (("--duty", args.duty), ("--freq", args.freq)):
        if vals and any(v <= 0 for v in vals):
            ap.error("%s values must be positive" % name)
    if any(d > 1.0 for d in duties):
        ap.error("--duty values must be <= 1.0")

    model = Model()
    if args.psi is not None:
        from nozzle_thrust import Conditions
        model = Model(conditions=Conditions(p_supply_gauge_psi=args.psi))

    # Sweep whichever variable has more values; the other becomes the series.
    x_key = "duty" if len(duties) >= len(freqs) else "frequency"
    series_key = "frequency" if x_key == "duty" else "duty"

    n_series = len(freqs) if series_key == "frequency" else len(duties)
    if n_series > 3:
        ap.error(
            "at most 3 values for the series variable (%s): a multi-line chart "
            "past three series is not colorblind-safe. Run separate sweeps."
            % series_key
        )

    rows = [
        pulse_point(model, f, d, args.duration)
        for f in freqs
        for d in duties
    ]
    steady = steady_mean(model, args.duration)

    print(_format_table(rows, steady))
    with plt.rc_context(_RC):
        out = plot_sweep(rows, steady, x_key, series_key, args.out,
                         model.conditions.p_supply_gauge_psi)
    print("\nWrote %s" % out)


if __name__ == "__main__":
    main()
