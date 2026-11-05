"""Plot range traces recorded by LaserLogging.py.

Run this with WSL python3, NOT python.exe: matplotlib lives in the conda env,
while the logger needs Windows Python to reach the COM port.

    python3 Laser_plotting.py hover1.csv
    python3 Laser_plotting.py trial1.csv trial2.csv --save trials.png
    python3 Laser_plotting.py hover1.csv --column raw_mm --no-stats
    python3 Laser_plotting.py hover1.csv --smooth 5 --rate
    python3 Laser_plotting.py hover1.csv --target 250
    python3 Laser_plotting.py hover1.csv --no-zero-base   # zoom to the noise
"""

import argparse
import math
import statistics
import sys

import matplotlib


COLUMNS = {'millis': 0, 'raw_mm': 1, 'range_mm': 2, 'status': 3}

AXIS_LABELS = {
    'range_mm': 'Range (mm)',
    'raw_mm': 'Raw range, pre-offset (mm)',
    'millis': 'Timestamp (ms)',
    'status': 'Range status code',
}

# Chip range status codes, as emitted by laser_logging.ino's readRangeStatus().
STATUS_NAMES = {
    0: 'valid',
    1: 'hardware fail', 2: 'hardware fail', 3: 'hardware fail',
    4: 'signal too low',
    5: 'below min range',
    6: 'phase/wraparound', 9: 'phase/wraparound',
    8: 'sigma too high', 10: 'sigma too high',
}

CLIP_SIGMAS = 3.0            # band half-width when isolating the steady level
CLIP_ROUNDS = 6              # sigma-clip iterations

# Categorical slots, assigned in fixed order and never cycled. Validated with
# the dataviz validate_palette.js: all six checks pass in both modes, worst
# adjacent CVD dE 9.1 light / 8.4 dark, normal-vision dE 19.6 light / 19.3 dark.
# Slot 8 is red here, not the gray used in Load_cell_plotting.py -- gray fails
# the chroma floor check (it reads as a disabled series, not a data series).
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100',
          '#e87ba4', '#008300', '#6250d6', '#e34948']

INK_PRIMARY = '#0b0b0b'
INK_SECONDARY = '#52514e'
INK_MUTED = '#7a7872'
SURFACE = '#fcfcfb'

# Dropouts are a defect, so they are marked in a status color, never a series
# slot. This is the "serious" step: distinct from all eight categorical hues.
INK_DROPOUT = '#c4320a'

# The target line is an annotation, not data, so it wears a text ink rather
# than a series hue. Using a categorical slot would imply it was a measured
# trace and would collide with whichever file happened to take that slot.
INK_TARGET = '#52514e'


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Plot VL53L0X CSVs recorded by LaserLogging.py.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    p.add_argument('files', nargs='+', help='one or more CSV files to plot')
    p.add_argument('--column', default='range_mm', choices=sorted(COLUMNS),
                   help='which column to plot on the y axis')
    p.add_argument('--save', default=None, metavar='PNG',
                   help='write the figure to a file instead of opening a window')
    p.add_argument('--title', default=None, help='override the plot title')
    p.add_argument('--dpi', type=int, default=120, help='resolution when saving')
    p.add_argument('--smooth', type=int, default=0, metavar='N',
                   help='centered rolling median over N samples (0 = off)')

    zero = p.add_mutually_exclusive_group()
    zero.add_argument('--zero-base', dest='zero_base', action='store_true',
                      default=None,
                      help='force the y axis to start at 0 mm')
    zero.add_argument('--no-zero-base', dest='zero_base', action='store_false',
                      help='let the y axis auto-scale to the data range')

    metres = p.add_mutually_exclusive_group()
    metres.add_argument('--metres-axis', dest='metres_axis', action='store_true',
                        default=None, help='force the metres axis on')
    metres.add_argument('--no-metres-axis', dest='metres_axis',
                        action='store_false', help='suppress the metres axis')

    p.add_argument('--target', type=float, default=None, metavar='MM',
                   help='draw a target distance and report error against it')
    p.add_argument('--no-stats', dest='stats', action='store_false',
                   help='suppress the mean/noise/dropout annotation')
    p.add_argument('--no-dropouts', dest='dropouts', action='store_false',
                   help='do not mark rejected samples')
    p.add_argument('--rate', action='store_true',
                   help='add a second panel showing sample interval')

    args = p.parse_args(argv)
    if args.smooth < 0:
        p.error('--smooth must be zero or positive')
    if args.dpi <= 0:
        p.error('--dpi must be positive')
    if args.target is not None:
        if args.target <= 0:
            p.error('--target must be positive')
        # A target is a distance, so it is meaningless against a status code
        # or a raw timestamp. Refuse rather than drawing a line at 250 on an
        # axis of status codes.
        if args.column not in ('range_mm', 'raw_mm'):
            p.error('--target only applies to --column range_mm or raw_mm')
    return args


# ------------------------------------------------------------------ loading

def load(path, column):
    """Return a trace dict from a logger CSV.

    Unlike the load-cell plotter, nan rows are not simply skipped. A nan here
    means the sensor rejected that individual sample mid-trial -- a dropout --
    so the timestamps are kept with a None value. That makes the gap visible
    in the line instead of silently closing it, which would otherwise draw a
    straight segment across missing data and imply a measurement that never
    happened.
    """
    index = COLUMNS[column]
    times, values, meta = [], [], {}
    dropout_times = []
    statuses = {}

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
            except ValueError:
                continue

            # Tally status codes regardless of which column we plot, so the
            # annotation can say WHY samples were rejected.
            try:
                code = int(fields[3])
                statuses[code] = statuses.get(code, 0) + 1
            except ValueError:
                pass

            raw_field = fields[index]
            try:
                value = float(raw_field)
            except ValueError:
                # 'nan' text -- a rejected sample.
                times.append(stamp)
                values.append(None)
                dropout_times.append(stamp)
                continue

            if math.isnan(value):
                times.append(stamp)
                values.append(None)
                dropout_times.append(stamp)
                continue

            times.append(stamp)
            values.append(value)

    # millis is time-since-boot; rebase to seconds from the first sample.
    if times:
        t0 = times[0]
        times = [(t - t0) / 1000.0 for t in times]
        dropout_times = [(t - t0) / 1000.0 for t in dropout_times]

    return {
        'path': path,
        'seconds': times,
        'values': values,
        'dropouts': dropout_times,
        'statuses': statuses,
        'meta': meta,
    }


def steady_stats(values):
    """Centre and spread of the steady portion of a trace.

    Seeded from the shorth -- the shortest interval containing half the samples
    -- which lands on the dominant population even when the sensor spends part
    of the trial elsewhere, then refined by a symmetric sigma-clip.

    Same approach as the load-cell plotter's baseline_stats, for the same
    reason: an asymmetric band biases the centre and halves the reported
    noise. Here it describes "the distance it was mostly holding" rather than
    an unloaded baseline.
    """
    clean = [v for v in values if v is not None]
    ordered = sorted(clean)
    count = len(ordered)
    if count < 4:
        return (statistics.fmean(clean) if clean else 0.0), 0.0, count

    half = count // 2
    start = min(range(count - half + 1),
                key=lambda i: ordered[i + half - 1] - ordered[i])
    band = ordered[start:start + half]
    centre = statistics.fmean(band)

    for _ in range(CLIP_ROUNDS):
        spread = statistics.pstdev(band) if len(band) > 1 else 0.0
        if spread <= 0:
            break
        kept = [v for v in clean if abs(v - centre) <= CLIP_SIGMAS * spread]
        if len(kept) < 2 or len(kept) == len(band):
            band = kept or band
            break
        band, centre = kept, statistics.fmean(kept)

    spread = statistics.pstdev(band) if len(band) > 1 else 0.0
    return centre, spread, len(band)


def intervals(seconds):
    """Sample-to-sample gaps in ms. Reveals timing budget and dropped frames."""
    return [(seconds[i] - seconds[i - 1]) * 1000.0
            for i in range(1, len(seconds))]


def error_stats(values, target):
    """Error of each valid sample against a target distance.

    Returns mean error (signed, so it reads as bias), its standard deviation,
    RMS error, and the largest absolute deviation.

    THREE THINGS WORTH KNOWING ABOUT THESE NUMBERS

    1. `mean` is signed on purpose. +4 mm means the sensor reads consistently
       long, -4 mm consistently short. A mean of absolute errors would hide
       that direction, and direction is exactly what tells you whether to
       adjust the offset.

    2. `sd` is the standard deviation OF THE ERROR, which is numerically
       identical to the standard deviation of the raw values. Subtracting a
       constant shifts a distribution without changing its spread. It is
       reported because it was asked for and because it is the right measure
       of repeatability -- but it carries no information about whether the
       sensor is actually near the target. Only `mean` does.

    3. `rms` is the one that answers "how far off am I, overall". It combines
       bias and scatter: rms^2 = mean^2 + sd^2. A sensor with +10 mm bias and
       1 mm noise has the same sd as one with 0 mm bias and 1 mm noise, but
       ten times the RMS error. If you want a single number, use this one.
    """
    errors = [v - target for v in values if v is not None]
    if not errors:
        return None

    mean = statistics.fmean(errors)
    sd = statistics.pstdev(errors) if len(errors) > 1 else 0.0
    rms = math.sqrt(statistics.fmean([e * e for e in errors]))

    return {
        'mean': mean,
        'sd': sd,
        'rms': rms,
        'max_abs': max(abs(e) for e in errors),
        'samples': len(errors),
        'within_5': 100.0 * sum(1 for e in errors if abs(e) <= 5.0) / len(errors),
    }


def summarize(trace, target=None):
    """Range, noise, dropout rate and cadence for one trace."""
    values = trace['values']
    seconds = trace['seconds']
    clean = [v for v in values if v is not None]

    if not clean:
        return None

    centre, spread, kept = steady_stats(values)
    gaps = intervals(seconds)

    return {
        'error': error_stats(values, target) if target is not None else None,
        'min': min(clean),
        'max': max(clean),
        'steady': centre,
        'noise': spread,
        'steady_samples': kept,
        'mean': statistics.fmean(clean),
        'samples': len(values),
        'valid': len(clean),
        'dropouts': len(trace['dropouts']),
        'median_dt': statistics.median(gaps) if gaps else 0.0,
        'rate': (1000.0 / statistics.median(gaps)) if gaps else 0.0,
    }


def smoothed(values, window):
    """Centered rolling MEDIAN, shrinking the window at the ends.

    A median, not a mean, and deliberately: VL53L0X errors are isolated gross
    outliers rather than Gaussian noise, so a mean would smear one bad sample
    across the whole window while a median rejects it outright. This matches
    the filter in the ESP-IDF driver's vl53l0x_iface.c.

    None values (dropouts) are excluded from each window rather than treated
    as zero, and a window with nothing valid in it stays None.
    """
    if window < 2:
        return values
    half = window // 2
    out = []
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        chunk = [v for v in values[lo:hi] if v is not None]
        out.append(statistics.median(chunk) if chunk else None)
    return out


def segments(seconds, values):
    """Split into runs of consecutive valid samples, breaking at dropouts.

    matplotlib would happily draw a line straight across a None, so the trace
    is plotted as separate segments. The visible gap is the point: it shows
    where data is missing instead of inventing a straight path through it.
    """
    runs, run_t, run_v = [], [], []
    for t, v in zip(seconds, values):
        if v is None:
            if run_t:
                runs.append((run_t, run_v))
                run_t, run_v = [], []
        else:
            run_t.append(t)
            run_v.append(v)
    if run_t:
        runs.append((run_t, run_v))
    return runs


# ---------------------------------------------------------------- annotation

def stat_lines(name, stats, column, target=None):
    """Two to four compact annotation lines per file."""
    pad = ' ' * len(name)
    lines = []

    if column in ('range_mm', 'raw_mm'):
        lines.append(
            '%s  steady %.1f mm (%.3f m)   noise sd %.2f mm'
            % (name, stats['steady'], stats['steady'] / 1000.0, stats['noise']))
        lines.append(
            '%s  span %.0f-%.0f mm   %.1f Hz (%.0f ms)'
            % (pad, stats['min'], stats['max'], stats['rate'],
               stats['median_dt']))
    else:
        lines.append('%s  mean %.1f   span %.0f-%.0f'
                     % (name, stats['mean'], stats['min'], stats['max']))
        lines.append('%s  %.1f Hz (%.0f ms)'
                     % (pad, stats['rate'], stats['median_dt']))

    # Error against the target, when one was given. The signed mean leads,
    # because its sign is the actionable part: it is the offset correction you
    # would apply, negated.
    err = stats.get('error')
    if err is not None and target is not None:
        lines.append(
            '%s  vs %g mm target: mean error %+.2f mm   sd %.2f mm   rms %.2f mm'
            % (pad, target, err['mean'], err['sd'], err['rms']))
        lines.append(
            '%s  worst %.2f mm off   %.0f%% within +/-5 mm   n=%d'
            % (pad, err['max_abs'], err['within_5'], err['samples']))

    if stats['dropouts']:
        pct = 100.0 * stats['dropouts'] / stats['samples']
        lines.append('%s  %d dropout(s), %.1f%% of %d samples'
                     % (pad, stats['dropouts'], pct, stats['samples']))
    return lines


def status_summary(traces):
    """Describe the non-valid status codes seen across all traces."""
    tally = {}
    for t in traces:
        for code, n in t['statuses'].items():
            if code == 0:
                continue
            tally[code] = tally.get(code, 0) + n
    if not tally:
        return None

    parts = []
    for code in sorted(tally, key=lambda c: -tally[c]):
        parts.append('%s x%d' % (STATUS_NAMES.get(code, 'code %d' % code),
                                 tally[code]))
    return 'rejected: ' + ', '.join(parts)


def subtitle_for(traces):
    """Describe the calibration and profile behind the traces."""
    metas = [t['meta'] for t in traces]
    offsets = {m['offset_mm'] for m in metas if 'offset_mm' in m}
    profiles = {m['profile'] for m in metas if 'profile' in m}

    parts = []
    if len(offsets) == 1:
        parts.append('%s mm offset' % offsets.pop())
    elif offsets:
        parts.append('mixed offsets')

    if len(profiles) == 1:
        parts.append(profiles.pop())
    elif profiles:
        parts.append('mixed profiles')

    return '   ·   '.join(parts) if parts else None


# ---------------------------------------------------------------------- main

def style_axis(ax, xlabel, ylabel):
    ax.set_facecolor(SURFACE)
    if xlabel:
        ax.set_xlabel(xlabel, color=INK_SECONDARY)
    ax.set_ylabel(ylabel, color=INK_SECONDARY)
    ax.grid(True, color=INK_MUTED, alpha=0.18, linewidth=0.7)
    ax.tick_params(colors=INK_SECONDARY, labelsize=9)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_MUTED)


def main(argv=None):
    args = parse_args(argv)
    if args.save:
        matplotlib.use('Agg')          # must precede the pyplot import
    import matplotlib.pyplot as plt

    traces = []
    for path in args.files:
        try:
            trace = load(path, args.column)
        except OSError as exc:
            print('Could not read %s: %s' % (path, exc))
            continue

        if not trace['seconds']:
            print('No usable data in %s.' % path)
            continue

        if not any(v is not None for v in trace['values']):
            print("Every '%s' value in %s is nan." % (args.column, path))
            print('  The sensor rejected every sample. Plot --column raw_mm')
            print('  to see what it was reading before validation.')
            continue

        traces.append(trace)

    if not traces:
        print('Nothing to plot.')
        return 1

    # The cadence panel is a second measure on a different scale, so it gets
    # its own axes rather than a second y-axis on the first plot.
    if args.rate:
        fig, (ax, ax_rate) = plt.subplots(
            2, 1, figsize=(10, 7), sharex=True,
            gridspec_kw={'height_ratios': [3, 1]})
    else:
        fig, ax = plt.subplots(figsize=(10, 5.5))
        ax_rate = None

    fig.patch.set_facecolor(SURFACE)

    # The target goes down first, at the lowest z-order, so every trace draws
    # over it. Dashed and in a text ink rather than a series hue: it is a
    # reference the operator chose, not data the sensor produced, and that
    # distinction should be visible at a glance.
    if args.target is not None:
        ax.axhline(args.target, color=INK_TARGET, linewidth=1.2,
                   linestyle='--', alpha=0.9, zorder=1,
                   label='target %g mm' % args.target)

    annotations = []
    for i, trace in enumerate(traces):
        color = SERIES[i % len(SERIES)]
        seconds, values = trace['seconds'], trace['values']
        stats = summarize(trace, args.target)
        # Earlier series sit on top: a flat trace would otherwise vanish under
        # a later trace's level, which can overlap it exactly.
        depth = 3 + (len(traces) - i)

        plot_values = smoothed(values, args.smooth) if args.smooth >= 2 else values

        if args.smooth >= 2:
            # Faded raw trace underneath, so smoothing cannot hide detail.
            for run_t, run_v in segments(seconds, values):
                ax.plot(run_t, run_v, color=color, linewidth=0.8, alpha=0.25,
                        zorder=depth - 1)

        # One label for the whole series, applied to the first segment only,
        # so a trace broken by dropouts still gets a single legend entry.
        labelled = False
        for run_t, run_v in segments(seconds, plot_values):
            ax.plot(run_t, run_v, color=color,
                    linewidth=1.8 if args.smooth >= 2 else 1.5,
                    label=None if labelled else trace['path'],
                    zorder=depth)
            labelled = True

        # Dropouts as rug ticks along the bottom: they mark WHERE data is
        # missing without implying a y-value the sensor never reported.
        if args.dropouts and trace['dropouts']:
            ax.plot(trace['dropouts'],
                    [0.012 + 0.016 * i] * len(trace['dropouts']),
                    transform=ax.get_xaxis_transform(),
                    linestyle='none', marker='|', markersize=7,
                    markeredgewidth=1.2, color=INK_DROPOUT,
                    alpha=0.75, zorder=2,
                    label=None if i else 'rejected sample')

        if args.stats and stats:
            annotations.extend(
                stat_lines(trace['path'], stats, args.column, args.target))

        if ax_rate is not None:
            gaps = intervals(seconds)
            if gaps:
                ax_rate.plot(seconds[1:], gaps, color=color, linewidth=1.0,
                             zorder=depth)

    style_axis(ax, None if ax_rate is not None else 'Time (s)',
               AXIS_LABELS[args.column])

    if ax_rate is not None:
        style_axis(ax_rate, 'Time (s)', 'Sample gap (ms)')

    # --- Zero-base the distance axis ------------------------------------
    # On by default for range_mm and raw_mm, because 0 mm is a true zero: a
    # distance of 0 means something physical, so the bar from 0 to the reading
    # is proportional to the quantity. Auto-scaling to the data instead picks
    # an arbitrary floor, which exaggerates small movements -- a hover holding
    # 265 +/- 3 mm looks violently unstable on an axis spanning 258 to 270.
    #
    # It is deliberately NOT applied to `status` (a code, where 0 has no
    # magnitude) or `millis` (already rebased to zero).
    #
    # --no-zero-base exists because the opposite problem is real: once the
    # axis runs 0 to 420, a 2 mm noise band is invisible. Use it when you are
    # looking at noise rather than at absolute distance.
    zero_base = args.zero_base
    if zero_base is None:
        zero_base = args.column in ('range_mm', 'raw_mm')

    if zero_base and args.column in ('range_mm', 'raw_mm'):
        _, auto_high = ax.get_ylim()
        # Keep matplotlib's headroom above the data; only the floor is pinned.
        ax.set_ylim(0.0, auto_high)

    # A metres axis is the same measure rescaled, not a second series, so this
    # is not a dual-axis chart. Cadence gets its own panel instead.
    want_metres = args.metres_axis
    if want_metres is None:
        want_metres = args.column in ('range_mm', 'raw_mm')
    if want_metres and args.column in ('range_mm', 'raw_mm'):
        secondary = ax.secondary_yaxis(
            'right', functions=(lambda mm: mm / 1000.0,
                                lambda m: m * 1000.0))
        secondary.set_ylabel('Range (m)', color=INK_SECONDARY)
        secondary.tick_params(colors=INK_SECONDARY, labelsize=9)
        secondary.spines['right'].set_color(INK_MUTED)

    title = args.title or 'VL53L0X %s' % args.column.replace('_mm', '')
    if len(traces) == 1 and not args.title:
        title = '%s — %s' % (title, traces[0]['path'])
    ax.set_title(title, color=INK_PRIMARY, fontsize=13, loc='left', pad=20)

    subtitle = subtitle_for(traces)
    if subtitle:
        ax.text(0.0, 1.015, subtitle, transform=ax.transAxes,
                color=INK_SECONDARY, fontsize=9, va='bottom')

    # A legend appears for two or more series so identity is never carried by
    # color alone. With one series the title names it, but a dropout marker
    # still needs explaining, so the legend appears for that too.
    handles, labels = ax.get_legend_handles_labels()
    if len(traces) > 1 or len(labels) > 1:
        # Reserve headroom so the legend box cannot sit on top of the traces.
        # Without this the legend covers data whenever the trace runs high on
        # the right-hand side, which for a hover trial is the common case.
        #
        # The fraction is of the FINAL span, so solving for the new top keeps
        # the floor fixed -- important now that the floor is pinned to 0 for
        # distance axes. Inflating the whole span instead (the obvious
        # `span / (1 - frac)`) would push the top up by a multiple of the
        # distance from 0 rather than of the data's own extent, which on a
        # zero-based axis leaves a large empty band above the traces.
        # The legend occupies a fixed fraction of the RENDERED box, so the
        # space reserved for it has to be that fraction of the axis span --
        # not of the data's own height. Reserving against the data extent
        # instead (tried, and wrong) under-reserves on a zero-based axis,
        # because the box is taller than the data: the legend then clips the
        # topmost trace.
        #
        # Solving for the new top keeps the floor fixed, which matters now
        # that the floor is pinned to 0 for distance axes.
        frac = 0.17 + 0.035 * len(labels)
        low, high = ax.get_ylim()
        ax.set_ylim(low, low + (high - low) / (1.0 - frac))

        # Series entries first, then the annotations -- the target line and
        # the dropout marker describe the data rather than being series, so
        # they read last. matplotlib orders by plot call, which would
        # otherwise interleave them between the files.
        def is_annotation(label):
            return label == 'rejected sample' or label.startswith('target ')

        order = ([i for i, lab in enumerate(labels) if not is_annotation(lab)]
                 + [i for i, lab in enumerate(labels) if is_annotation(lab)])

        legend = ax.legend([handles[i] for i in order],
                           [labels[i] for i in order],
                           loc='upper right', frameon=True, fontsize=9)
        legend.get_frame().set_facecolor(SURFACE)
        legend.get_frame().set_edgecolor(INK_MUTED)
        legend.get_frame().set_linewidth(0.7)
        for text in legend.get_texts():
            text.set_color(INK_SECONDARY)

    fig.tight_layout()

    # The stats block goes below the axes as a caption: inside the plot it
    # covers the trace and can hide detail underneath itself.
    status_line = status_summary(traces) if args.stats else None
    caption = list(annotations)
    if status_line:
        caption.append(status_line)

    if caption:
        rows = len(caption)
        fig.subplots_adjust(bottom=0.10 + 0.028 * rows)
        fig.text(0.013, 0.028 * rows * 0.34, '\n'.join(caption),
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
