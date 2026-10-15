"""
Plot valve control input and resulting nozzle thrust against time.

Two stacked panels share one time axis. Control input is dimensionless (0-1)
and thrust is in newtons, so these are deliberately separate panels rather
than a dual-axis chart -- two y-scales on one frame invite false correlation.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

from nozzle_thrust import Model, Result, Schedule, default_schedule, simulate

# Reference palette, light mode. Slot 1 blue = thrust (the response),
# slot 2 orange = command (the input). Both are documented as passing every
# hard gate in both modes; used unchanged.
COLOR_THRUST = "#2a78d6"
COLOR_COMMAND = "#eb6834"
COLOR_SURFACE = "#fcfcfb"
COLOR_TEXT = "#0b0b0b"
COLOR_TEXT_SECONDARY = "#52514e"
COLOR_TEXT_MUTED = "#7a7974"
COLOR_GRID = "#e4e3df"
COLOR_BAND = "#f2f1ed"

_RC = {
    "figure.facecolor": COLOR_SURFACE,
    "axes.facecolor": COLOR_SURFACE,
    "savefig.facecolor": COLOR_SURFACE,
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "Helvetica Neue", "Arial", "DejaVu Sans"],
    "text.color": COLOR_TEXT,
    "axes.labelcolor": COLOR_TEXT_SECONDARY,
    "axes.edgecolor": COLOR_GRID,
    "xtick.color": COLOR_TEXT_SECONDARY,
    "ytick.color": COLOR_TEXT_SECONDARY,
    "xtick.labelcolor": COLOR_TEXT_SECONDARY,
    "ytick.labelcolor": COLOR_TEXT_SECONDARY,
    "axes.grid": True,
    "grid.color": COLOR_GRID,
    "grid.linewidth": 0.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.linewidth": 0.8,
    "font.size": 10,
}


def _phase_subtitle(schedule: Schedule) -> str:
    parts = []
    for phase, t0, t1 in schedule.phase_spans():
        if phase.mode == "pulse":
            parts.append(
                "%gs %gHz @ %d%% duty"
                % (t1 - t0, phase.frequency, round(phase.duty_cycle * 100))
            )
        elif phase.mode == "open":
            parts.append("%gs open" % (t1 - t0))
        else:
            parts.append("%gs closed" % (t1 - t0))
    return "  ->  ".join(parts)


def _annotate_phases(ax, schedule: Schedule, shade: bool) -> None:
    """Shade alternating phases and label each along the top of the panel."""
    for i, (phase, t0, t1) in enumerate(schedule.phase_spans()):
        if shade and i % 2 == 1:
            ax.axvspan(t0, t1, color=COLOR_BAND, zorder=0, linewidth=0)
        if t0 > 0:
            ax.axvline(
                t0, color=COLOR_GRID, linewidth=0.8, zorder=1, linestyle=(0, (4, 3))
            )


def _phase_headers(ax, schedule: Schedule) -> None:
    for phase, t0, t1 in schedule.phase_spans():
        label = phase.label or phase.mode.title()
        if phase.mode == "pulse" and not phase.label:
            label = "Pulsed"
        ax.annotate(
            label,
            xy=(0.5 * (t0 + t1), 1.0),
            xycoords=("data", "axes fraction"),
            xytext=(0, 7),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=9,
            color=COLOR_TEXT_SECONDARY,
            fontweight="semibold",
        )


def _first_pulse_phase(schedule: Schedule):
    for phase, t0, t1 in schedule.phase_spans():
        if phase.mode == "pulse" and phase.frequency > 0:
            return phase, t0, t1
    return None


def _detail_window(schedule: Schedule, cycles: float = 4.0):
    """A short time window inside the pulse phase, a few cycles wide.

    At 20 Hz over 5 s the overview panel draws 100 cycles into a few hundred
    pixels, which reads as a solid block. The detail window is what makes the
    pulse shape legible.
    """
    found = _first_pulse_phase(schedule)
    if found is None:
        return None
    phase, t0, t1 = found
    period = 1.0 / phase.frequency
    width = min(cycles * period, t1 - t0)
    # Start a couple of cycles in, so the window shows settled cycling.
    start = min(t0 + 2.0 * period, t1 - width)
    return max(t0, start), max(t0, start) + width


def _draw_detail(ax, result: Result, window: tuple[float, float]) -> None:
    """Thrust over a few pulse cycles, with valve-open intervals shaded.

    The command is shown as shading rather than a second y-axis: the panel has
    one scale (newtons) and the shading carries the on/off state.
    """
    t0, t1 = window
    t = result.time
    m = (t >= t0) & (t <= t1)

    # Shade commanded-open intervals from the command signal itself.
    cmd = result.command[m]
    tw = t[m]
    if tw.size:
        open_mask = cmd > 0.5
        edges = np.flatnonzero(np.diff(open_mask.astype(int)))
        bounds = np.concatenate(([0], edges + 1, [tw.size - 1]))
        for a, b in zip(bounds[:-1], bounds[1:]):
            if open_mask[a]:
                ax.axvspan(tw[a], tw[min(b, tw.size - 1)],
                           color=COLOR_COMMAND, alpha=0.11, linewidth=0, zorder=1)

    # No area fill here: it would sit on top of the command shading and the two
    # translucent layers mix to a muddy gray. The 2px line carries the thrust.
    ax.plot(t[m], result.thrust[m], color=COLOR_THRUST, linewidth=2.0, zorder=3)

    peak = float(result.thrust.max()) if result.thrust.size else 1.0
    ax.set_ylim(0.0, max(peak * 1.26, 1e-6))
    ax.set_xlim(t0, t1)
    ax.set_ylabel("Thrust  (N)", labelpad=8)
    ax.set_xlabel("Time  (s)  -  detail window", labelpad=6)
    ax.grid(axis="x", visible=False)

    phase = _first_pulse_phase(result.schedule)[0]
    ax.set_title(
        "Detail: %g Hz @ %d%% duty  (valve-open intervals shaded)"
        % (phase.frequency, round(phase.duty_cycle * 100)),
        loc="left", fontsize=9, color=COLOR_TEXT_SECONDARY,
        fontweight="semibold", pad=8,
    )


def plot_result(
    result: Result,
    path: str | Path | None = None,
    title: str = "Cold-gas nozzle thrust response to valve actuation",
    show_pressure: bool = False,
    show_detail: bool = True,
    annotate: bool = True,
):
    """Render control input and thrust against a shared time axis.

    show_pressure adds a panel with plenum stagnation pressure, the state
    variable that explains the thrust lag. show_detail adds a zoomed window
    of a few pulse cycles, without which a fast pulse train is just a block.
    """
    schedule = result.schedule
    model = result.model

    window = _detail_window(schedule) if show_detail else None

    heights = [1.0, 1.9]
    if show_pressure:
        heights = [1.0, 1.6, 1.1]
    if window is not None:
        heights = heights + [1.35]
    n_panels = len(heights)

    fig, axes = plt.subplots(
        n_panels,
        1,
        figsize=(11.0, 2.9 + 2.0 * n_panels),
        gridspec_kw={"height_ratios": heights, "hspace": 0.34},
    )
    ax_cmd, ax_thrust = axes[0], axes[1]
    ax_press = axes[2] if show_pressure else None
    ax_detail = axes[-1] if window is not None else None

    # The overview panels share the full-run time axis; the detail panel does not.
    overview = [ax for ax in axes if ax is not ax_detail]
    for ax in overview[:-1]:
        ax.sharex(overview[-1])
        ax.tick_params(labelbottom=False)

    t = result.time

    # --- Panel 1: commanded valve state -------------------------------------
    _annotate_phases(ax_cmd, schedule, shade=annotate)
    ax_cmd.plot(
        t, result.command, color=COLOR_COMMAND, linewidth=2.0, zorder=3,
        drawstyle="steps-post", solid_joinstyle="miter",
    )
    ax_cmd.set_ylim(-0.12, 1.22)
    ax_cmd.set_yticks([0.0, 1.0])
    ax_cmd.set_yticklabels(["Closed", "Open"])
    ax_cmd.set_ylabel("Valve command", labelpad=8)
    ax_cmd.grid(axis="x", visible=False)
    if annotate:
        _phase_headers(ax_cmd, schedule)

    # --- Panel 2: thrust ----------------------------------------------------
    _annotate_phases(ax_thrust, schedule, shade=annotate)
    ax_thrust.fill_between(
        t, 0.0, result.thrust, color=COLOR_THRUST, alpha=0.13, linewidth=0, zorder=2
    )
    ax_thrust.plot(t, result.thrust, color=COLOR_THRUST, linewidth=2.0, zorder=3)

    peak = float(result.thrust.max()) if result.thrust.size else 0.0
    ax_thrust.set_ylim(0.0, max(peak * 1.26, 1e-6))
    ax_thrust.set_ylabel("Thrust  (N)", labelpad=8)
    ax_thrust.grid(axis="x", visible=False)

    if annotate and peak > 0:
        # Direct-label the steady plateau and the pulsed mean -- the two
        # numbers a reader actually takes away. Never a label per point.
        stats = result.phase_stats()
        for st in stats:
            phase = st["phase"]
            if phase.mode == "open":
                ax_thrust.annotate(
                    "%.2f N steady" % st["peak_thrust"],
                    xy=(0.5 * (st["t_start"] + st["t_end"]), st["peak_thrust"]),
                    xytext=(0, 9),
                    textcoords="offset points",
                    ha="center",
                    fontsize=9.5,
                    color=COLOR_TEXT,
                    fontweight="semibold",
                )
            elif phase.mode == "pulse" and st["mean_thrust"] > 0:
                ax_thrust.hlines(
                    st["mean_thrust"], st["t_start"], st["t_end"],
                    color=COLOR_THRUST, linewidth=1.3, linestyle=(0, (5, 3)),
                    zorder=4,
                )
                ax_thrust.annotate(
                    "%.2f N mean" % st["mean_thrust"],
                    xy=(st["t_end"], st["mean_thrust"]),
                    xytext=(-6, 8),
                    textcoords="offset points",
                    ha="right",
                    fontsize=9.5,
                    color=COLOR_TEXT,
                    fontweight="semibold",
                )

    # --- Panel 3 (optional): plenum pressure --------------------------------
    if ax_press is not None:
        _annotate_phases(ax_press, schedule, shade=annotate)
        pa = model.conditions.p_ambient
        ax_press.plot(
            t, result.plenum_pressure / 1e5, color=COLOR_THRUST,
            linewidth=1.8, zorder=3,
        )
        ax_press.axhline(
            model.conditions.p_supply / 1e5, color=COLOR_TEXT_MUTED,
            linewidth=1.0, linestyle=(0, (5, 3)), zorder=2,
        )
        ax_press.annotate(
            "supply %.1f bar" % (model.conditions.p_supply / 1e5),
            xy=(t[-1], model.conditions.p_supply / 1e5),
            xytext=(-4, -12), textcoords="offset points",
            ha="right", fontsize=8.5, color=COLOR_TEXT_MUTED,
        )
        ax_press.set_ylim(0.0, model.conditions.p_supply / 1e5 * 1.18)
        ax_press.set_ylabel("Plenum  (bar abs)", labelpad=8)
        ax_press.grid(axis="x", visible=False)

    # --- Shared x axis (overview panels only) -------------------------------
    ax_bottom = overview[-1]
    ax_bottom.set_xlabel("Time  (s)", labelpad=6)
    ax_bottom.set_xlim(0.0, schedule.duration)
    span = schedule.duration
    ax_bottom.xaxis.set_major_locator(MultipleLocator(5 if span > 12 else 1))
    ax_bottom.xaxis.set_minor_locator(MultipleLocator(1 if span > 12 else 0.2))

    # --- Detail panel -------------------------------------------------------
    if ax_detail is not None:
        _draw_detail(ax_detail, result, window)
        # Mark the detail window on the thrust overview so the zoom is locatable.
        for edge in window:
            ax_thrust.axvline(edge, color=COLOR_TEXT_MUTED, linewidth=1.0,
                              linestyle=(0, (3, 2)), zorder=5)

    # --- Titles -------------------------------------------------------------
    fig.suptitle(title, x=0.062, y=0.982, ha="left", fontsize=14.5,
                 fontweight="semibold", color=COLOR_TEXT)
    subtitle = "%s   |   %.0f psig N2, sea level   |   total impulse %.2f N*s, %.1f g N2" % (
        _phase_subtitle(schedule),
        model.conditions.p_supply_gauge_psi,
        result.total_impulse,
        result.propellant_used * 1000.0,
    )
    fig.text(0.062, 0.945, subtitle, ha="left", fontsize=9.5, color=COLOR_TEXT_SECONDARY)

    fig.subplots_adjust(left=0.098, right=0.975, top=0.905, bottom=0.068)

    if path is not None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=200)
        plt.close(fig)
        return path
    return fig


def summary_table(result: Result) -> str:
    """Plain-text table view of the per-phase numbers (accessibility pass)."""
    rows = [
        "Phase            Window (s)     Mean F (N)   Peak F (N)   Impulse (N*s)   N2 (g)   Isp (s)",
        "-" * 94,
    ]
    for st in result.phase_stats():
        phase = st["phase"]
        label = phase.label or phase.mode.title()
        if phase.mode == "pulse":
            label = "%s %gHz/%d%%" % (label, phase.frequency, round(phase.duty_cycle * 100))
        isp = "    -  " if st["propellant"] <= 1e-9 else "%7.1f" % st["isp"]
        rows.append(
            "%-16s %5.2f-%-7.2f %10.3f %12.3f %15.3f %8.2f %s"
            % (label[:16], st["t_start"], st["t_end"], st["mean_thrust"],
               st["peak_thrust"], st["impulse"], st["propellant"] * 1000.0, isp)
        )
    rows.append("-" * 94)
    rows.append(
        "%-16s %5.2f-%-7.2f %10.3f %12.3f %15.3f %8.2f %7.1f"
        % ("TOTAL", 0.0, result.schedule.duration,
           result.total_impulse / result.schedule.duration,
           float(result.thrust.max()), result.total_impulse,
           result.propellant_used * 1000.0,
           result.total_impulse / (result.propellant_used * 9.80665))
    )
    return "\n".join(rows)


def main() -> None:
    with plt.rc_context(_RC):
        result = simulate(Model(), default_schedule(), sample_dt=200e-6)
        out = Path(__file__).parent / "thrust_profile.png"
        plot_result(result, out, show_pressure=True)
    print(summary_table(result))
    print("\nWrote %s" % out)


if __name__ == "__main__":
    main()
