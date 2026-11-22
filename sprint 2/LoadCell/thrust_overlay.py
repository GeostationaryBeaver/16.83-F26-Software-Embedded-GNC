"""Overlay the two 11 Nov thrust tests with peaks aligned at t = 0.

Both traces are rebuilt from the 'counts' column against one constant, because
the two runs logged counts_per_newton values differing by 2.1x, so their
force_N columns are not directly comparable.

    python3 thrust_overlay.py
    python3 thrust_overlay.py --window -20 60
"""

import argparse
import math
import statistics
import sys

import matplotlib

# Eight of the eleven 11 Nov runs calibrated near this value, and run 164511
# confirms it: a 1011 g mass read 9.955 N where 9.915 N is expected (+0.4%).
# The 16:07-16:13 runs used ~8586 counts/N, inflating their forces ~2.3x.
REFERENCE_CPN = 19511.0
G0 = 9.80665

TESTS = [('Thrust Test 1', 'load_cell_20261111_161316.csv', '#2a78d6'),
         ('Thrust Test 2', 'load_cell_20261111_170413.csv', '#eb6834')]

INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#7a7872'
SURFACE = '#fcfcfb'


def load(path, rescale=True):
    """Return (seconds, newtons), dropping failed HX711 reads."""
    times, values = [], []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('millis'):
                continue
            parts = line.split(',')
            if len(parts) != 4:
                continue
            try:
                stamp, raw, counts = int(parts[0]), int(parts[1]), int(parts[2])
                force = float(parts[3])
            except ValueError:
                continue
            if raw == -1 or math.isnan(force):   # sensor-fault signature
                continue
            times.append(stamp)
            values.append(counts / REFERENCE_CPN if rescale else force)
    origin = times[0]
    return [(t - origin) / 1000.0 for t in times], values


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--save', default='plots/thrust_overlay.png')
    ap.add_argument('--window', nargs=2, type=float, default=[-20.0, 90.0],
                    metavar=('BEFORE', 'AFTER'))
    ap.add_argument('--as-logged', action='store_true',
                    help='skip the re-scale; use force_N as recorded')
    ap.add_argument('--dpi', type=int, default=140)
    args = ap.parse_args(argv)

    if args.save != '-':
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    before, after = args.window
    fig, ax = plt.subplots(figsize=(11, 6))
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    caption = []
    for i, (label, path, color) in enumerate(TESTS):
        seconds, values = load(path, not args.as_logged)
        peak = max(values)
        shift = seconds[values.index(peak)]
        aligned = [(s - shift, v) for s, v in zip(seconds, values)]

        pre = [v for s, v in aligned if s < -2.0]
        base = statistics.median(pre) if pre else 0.0

        kept = [(s, v) for s, v in aligned if before <= s <= after]
        xs = [s for s, _ in kept]
        ys = [v for _, v in kept]

        depth = 3 + len(TESTS) - i          # earlier series drawn on top
        ax.plot(xs, ys, color=color, linewidth=1.5, label=label, zorder=depth)
        ax.plot([0.0], [peak], marker='o', markersize=6, color=color,
                markeredgecolor=SURFACE, markeredgewidth=1.5,
                zorder=depth + len(TESTS))

        half = next((s for s, v in kept
                     if s > 0 and v <= base + (peak - base) * 0.5), None)
        caption.append(
            '%-14s %-34s peak %6.2f N (%5.0f g)   baseline %+5.2f N%s'
            % (label, path, peak, peak / G0 * 1000.0, base,
               '   half-decay %4.1f s' % half if half else ''))

    ax.axvline(0.0, color=MUTED, linewidth=1.0, linestyle='--', alpha=0.7,
               zorder=1)
    ax.axhline(0.0, color=MUTED, linewidth=0.9, linestyle=':', alpha=0.7,
               zorder=1)
    ax.text(0.0, 1.002, ' peak', transform=ax.get_xaxis_transform(),
            color=MUTED, fontsize=8, ha='left', va='bottom')

    ax.set_xlabel('Time from peak (s)', color=INK2)
    ax.set_ylabel('Force (N)', color=INK2)
    ax.set_xlim(before, after)
    ax.grid(True, color=MUTED, alpha=0.18, linewidth=0.7)
    ax.tick_params(colors=INK2, labelsize=9)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)

    grams = ax.secondary_yaxis('right', functions=(
        lambda n: n / G0 * 1000.0, lambda g: g * G0 / 1000.0))
    grams.set_ylabel('Equivalent mass (g)', color=INK2)
    grams.tick_params(colors=INK2, labelsize=9)
    grams.spines['right'].set_color(MUTED)

    ax.set_title('Thrust tests — 11 Nov 2026, peaks aligned', color=INK,
                 fontsize=13, loc='left', pad=20)
    ax.text(0.0, 1.015,
            'force_N as logged (calibrations differ 2.1x)' if args.as_logged
            else 'both traces rebuilt from counts at %.0f counts/N' % REFERENCE_CPN,
            transform=ax.transAxes, color=INK2, fontsize=9, va='bottom')

    legend = ax.legend(loc='upper right', frameon=True, fontsize=9)
    legend.get_frame().set_facecolor(SURFACE)
    legend.get_frame().set_edgecolor(MUTED)
    legend.get_frame().set_linewidth(0.7)
    for text in legend.get_texts():
        text.set_color(INK2)

    fig.tight_layout()
    fig.subplots_adjust(bottom=0.10 + 0.030 * len(caption))
    fig.text(0.013, 0.030 * len(caption) * 0.34, '\n'.join(caption),
             fontsize=8, family='monospace', color=INK2, va='center')

    if args.save == '-':
        plt.show()
    else:
        fig.savefig(args.save, dpi=args.dpi, bbox_inches='tight',
                    facecolor=SURFACE)
        print('Saved %s' % args.save)
    return 0


if __name__ == '__main__':
    sys.exit(main())
