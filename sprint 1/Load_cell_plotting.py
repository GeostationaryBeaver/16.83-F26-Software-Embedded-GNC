"""Plot force traces recorded by LoadCellLogging.py.

Run this with WSL python3, NOT python.exe: matplotlib lives in the conda env,
while the logger needs Windows Python to reach the COM port.

    python3 Load_cell_plotting.py test2.csv
    python3 Load_cell_plotting.py test1.csv test2.csv --save trials.png
    python3 Load_cell_plotting.py test2.csv --column counts --no-stats
"""

import argparse
import math
import statistics
import sys

import matplotlib


COLUMNS = {'millis': 0, 'raw': 1, 'counts': 2, 'force_N': 3}

AXIS_LABELS = {
    'force_N': 'Force (N)',
    'millis': 'Timestamp (ms)',
    'raw': 'Raw ADC counts',
    'counts': 'Net counts',
}

G0 = 9.80665                 # m/s^2, matches loadcell3.ino
CLIP_SIGMAS = 3.0            # band half-width when isolating the baseline
CLIP_ROUNDS = 6              # sigma-clip iterations

# Categorical slots, assigned in fixed order and never cycled. Validated with
# the dataviz validate_palette.js: all six checks pass, worst adjacent pair
# CVD dE 24.7 (protan), normal-vision dE 33.6.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100',
          '#e87ba4', '#008300', '#6250d6', '#8a8a8a']

INK_PRIMARY = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#7a7872'
SURFACE = '#fcfcfb'


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Plot load-cell CSVs recorded by LoadCellLogging.py.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    p.add_argument('files', nargs='+', help='one or more CSV files to plot')
    p.add_argument('--column', default='force_N', choices=sorted(COLUMNS),
                   help='which column to plot on the y axis')
    p.add_argument('--save', default=None, metavar='PNG',
                   help='write the figure to a file instead of opening a window')
    p.add_argument('--title', default=None, help='override the plot title')
    p.add_argument('--dpi', type=int, default=120, help='resolution when saving')
    p.add_argument('--smooth', type=int, default=0, metavar='N',
                   help='centered rolling mean over N samples (0 = off)')

    grams = p.add_mutually_exclusive_group()
    grams.add_argument('--grams-axis', dest='grams_axis', action='store_true',
                       default=None, help='force the grams axis on')
    grams.add_argument('--no-grams-axis', dest='grams_axis',
                       action='store_false', help='suppress the grams axis')

    p.add_argument('--no-stats', dest='stats', action='store_false',
                   help='suppress the peak/baseline/noise annotation')
    p.add_argument('--baseline', action='store_true',
                   help='draw a horizontal zero reference line')

    args = p.parse_args(argv)
    if args.smooth < 0:
        p.error('--smooth must be zero or positive')
    if args.dpi <= 0:
        p.error('--dpi must be positive')
    return args


# ------------------------------------------------------------------ loading

def load(path, column):
    """Return (seconds, values, metadata, skipped_nan) from a logger CSV."""
    index = COLUMNS[column]
    times, values, meta, skipped = [], [], {}, 0

    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # The logger writes '# key=value' metadata ahead of the header.
            if line.startswith('#'):
                body = line.lstrip('#').strip()
                if '=' in body:
                    key, _, value = body.partition('=')
                    value = value.strip()
                    if value.lower() != 'none':
                        meta[key.strip()] = value
                continue

            if line.startswith('millis'):
                continue

            fields = line.split(',')
            if len(fields) != 4:
                continue
            try:
                stamp = float(fields[0])
                value = float(fields[index])
            except (ValueError, IndexError):
                continue

            # float('nan') parses fine, so NaN must be filtered explicitly: an
            # uncalibrated run has force_N = nan on every row.
            if math.isnan(value):
                skipped += 1
                continue

            times.append(stamp)
            values.append(value)

    # millis is time-since-boot; rebase to seconds from the first sample.
    if times:
        t0 = times[0]
        times = [(t - t0) / 1000.0 for t in times]
    return times, values, meta, skipped


def baseline_stats(values):
    """Centre and spread of the unloaded portion of a trace.

    Seeded from the shorth -- the shortest interval containing half the samples
    -- which lands on the unloaded population even when much of the trial is
    loaded, then refined by a symmetric sigma-clip. The band has to be
    symmetric: simply taking the lowest quantile biases the centre downward by
    about one standard deviation and halves the reported noise.

    A median-absolute-deviation scale is not usable here, because force_N is
    written with three decimals while the noise is ~0.0014 N, so the MAD
    quantises to exactly zero.
    """
    ordered = sorted(values)
    count = len(ordered)
    if count < 4:
        return statistics.fmean(values), 0.0, count

    half = count // 2
    start = min(range(count - half + 1),
                key=lambda i: ordered[i + half - 1] - ordered[i])
    band = ordered[start:start + half]
    centre = statistics.fmean(band)

    for _ in range(CLIP_ROUNDS):
        spread = statistics.pstdev(band) if len(band) > 1 else 0.0
        if spread <= 0:
            break
        kept = [v for v in values if abs(v - centre) <= CLIP_SIGMAS * spread]
        if len(kept) < 2 or len(kept) == len(band):
            band = kept or band
            break
        band, centre = kept, statistics.fmean(kept)

    spread = statistics.pstdev(band) if len(band) > 1 else 0.0
    return centre, spread, len(band)


def summarize(seconds, values):
    """Peak, baseline and noise for one trace."""
    peak = max(values)
    centre, spread, kept = baseline_stats(values)

    return {
        'peak': peak,
        'peak_at': seconds[values.index(peak)],
        'baseline': centre,
        'noise': spread,
        'baseline_samples': kept,
        'mean': statistics.fmean(values),
        'samples': len(values),
    }


def smoothed(values, window):
    """Centered rolling mean, shrinking the window at the ends."""
    if window < 2:
        return values
    half = window // 2
    out = []
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        out.append(sum(values[lo:hi]) / (hi - lo))
    return out


# ---------------------------------------------------------------- annotation

def stat_lines(name, stats, column):
    """Two compact annotation lines per file."""
    pad = ' ' * len(name)
    if column == 'force_N':
        return [
            '%s  peak %.3f N (%.1f g) @ %.1f s'
            % (name, stats['peak'], stats['peak'] / G0 * 1000.0,
               stats['peak_at']),
            '%s  baseline %+.4f N   noise sd %.4f N (%.2f g)'
            % (pad, stats['baseline'], stats['noise'],
               stats['noise'] / G0 * 1000.0),
        ]
    return [
        '%s  peak %.0f @ %.1f s' % (name, stats['peak'], stats['peak_at']),
        '%s  baseline %+.0f   noise sd %.1f'
        % (pad, stats['baseline'], stats['noise']),
    ]


def subtitle_for(metas):
    """Describe the calibration behind the traces, if they report one."""
    grams = {m['calibration_grams'] for m in metas if 'calibration_grams' in m}
    counts = {m['counts_per_newton'] for m in metas if 'counts_per_newton' in m}
    if not grams and not counts:
        return None

    parts = []
    if len(grams) == 1:
        parts.append('%s g calibration mass' % grams.pop())
    elif grams:
        parts.append('mixed calibration masses')

    if len(counts) == 1:
        parts.append('%.0f counts/N' % float(counts.pop()))
    elif counts:
        spread = sorted(float(c) for c in counts)
        parts.append('%.0f-%.0f counts/N' % (spread[0], spread[-1]))
    return '   ·   '.join(parts)


# ---------------------------------------------------------------------- main

def main(argv=None):
    args = parse_args(argv)
    if args.save:
        matplotlib.use('Agg')          # must precede the pyplot import
    import matplotlib.pyplot as plt

    traces = []
    for path in args.files:
        try:
            seconds, values, meta, skipped = load(path, args.column)
        except OSError as exc:
            print('Could not read %s: %s' % (path, exc))
            continue

        if skipped:
            print('%s: skipped %d row(s) with %s=nan'
                  % (path, skipped, args.column))

        if not seconds:
            print("No usable '%s' data in %s." % (args.column, path))
            if args.column == 'force_N':
                print('  A run recorded without --grams has no calibration, so')
                print('  force_N is nan on every row. Re-run this with')
                print('  --column counts to see the uncalibrated output.')
            continue

        traces.append((path, seconds, values, meta))

    if not traces:
        print('Nothing to plot.')
        return 1

    fig, ax = plt.subplots(figsize=(10, 5.5))
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    annotations = []
    for i, (path, seconds, values, _meta) in enumerate(traces):
        color = SERIES[i % len(SERIES)]
        stats = summarize(seconds, values)
        # Earlier series sit on top: a flat trace would otherwise vanish under
        # a later trace's baseline, which overlaps it exactly.
        depth = 3 + (len(traces) - i)

        if args.smooth >= 2:
            # Faded raw trace underneath, so smoothing cannot hide detail.
            ax.plot(seconds, values, color=color, linewidth=0.8, alpha=0.25,
                    zorder=depth - 1)
            ax.plot(seconds, smoothed(values, args.smooth), color=color,
                    linewidth=1.8, label=path, zorder=depth)
        else:
            ax.plot(seconds, values, color=color, linewidth=1.5, label=path,
                    zorder=depth)

        # Selective direct labelling: mark the peak only, never every point.
        if args.stats:
            ax.plot([stats['peak_at']], [stats['peak']], marker='o',
                    markersize=5, color=color, markeredgecolor=SURFACE,
                    markeredgewidth=1.5, zorder=depth + len(traces))
            annotations.extend(stat_lines(path, stats, args.column))

    if args.baseline:
        ax.axhline(0.0, color=INK_MUTED, linewidth=1.0, linestyle=':', zorder=1)

    ax.set_xlabel('Time (s)', color=INK_SECONDARY)
    ax.set_ylabel(AXIS_LABELS[args.column], color=INK_SECONDARY)
    ax.grid(True, color=INK_MUTED, alpha=0.18, linewidth=0.7)
    ax.tick_params(colors=INK_SECONDARY, labelsize=9)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_MUTED)

    # A grams axis is the same measure rescaled, not a second series, so this is
    # not a dual-axis chart. Counts are plotted on their own figure instead.
    want_grams = args.grams_axis
    if want_grams is None:
        want_grams = args.column == 'force_N'
    if want_grams and args.column == 'force_N':
        secondary = ax.secondary_yaxis(
            'right', functions=(lambda n: n / G0 * 1000.0,
                                lambda g: g * G0 / 1000.0))
        secondary.set_ylabel('Equivalent mass (g)', color=INK_SECONDARY)
        secondary.tick_params(colors=INK_SECONDARY, labelsize=9)
        secondary.spines['right'].set_color(INK_MUTED)

    title = args.title or 'Load cell %s' % args.column.replace('_N', '')
    if len(traces) == 1 and not args.title:
        title = '%s — %s' % (title, traces[0][0])
    ax.set_title(title, color=INK_PRIMARY, fontsize=13, loc='left', pad=20)

    subtitle = subtitle_for([m for _, _, _, m in traces])
    if subtitle:
        ax.text(0.0, 1.015, subtitle, transform=ax.transAxes,
                color=INK_SECONDARY, fontsize=9, va='bottom')

    # One series is named by the title; two or more need a legend so identity is
    # never carried by color alone.
    if len(traces) > 1:
        legend = ax.legend(loc='upper right', frameon=True, fontsize=9)
        legend.get_frame().set_facecolor(SURFACE)
        legend.get_frame().set_edgecolor(INK_MUTED)
        legend.get_frame().set_linewidth(0.7)
        for text in legend.get_texts():
            text.set_color(INK_SECONDARY)

    fig.tight_layout()

    # The stats block goes below the axes as a caption: inside the plot it
    # covers the rising edges and can hide a peak marker underneath itself.
    if annotations:
        rows = len(annotations)
        fig.subplots_adjust(bottom=0.10 + 0.028 * rows)
        fig.text(0.013, 0.028 * rows * 0.34, '\n'.join(annotations),
                 fontsize=8.5, family='monospace', color=INK_SECONDARY,
                 va='center', ha='left')

    if args.save:
        fig.savefig(args.save, dpi=args.dpi, bbox_inches='tight',
                    facecolor=SURFACE)
        print('Saved %s' % args.save)
    else:
        plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
