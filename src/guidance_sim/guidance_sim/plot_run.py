"""Plot run CSVs produced by :mod:`guidance_sim.offline` or
:mod:`guidance_sim.engage`.

Two modes, chosen automatically from the header:

**Run mode** (``t,R,Rdot,...``) draws a five-panel figure for one or more runs
overlaid: the NE trajectory, range, LOS rate, signed latax against its
saturation bound, and heading error. Overlaying is the point -- the interesting
statements in guidance ("PN nulls the LOS rate, pursuit does not", "pursuit
saturates, PN does not") are comparisons, not single curves.

**Sweep mode** (``law,N,nu,...``) draws the capturability chart: miss distance
against speed ratio, one line per law and navigation ratio.

Frame reminder: the logs are NED, so Down is positive and an altitude of 50 m
appears as ``-50``. The trajectory panel therefore plots East on the horizontal
axis and North on the vertical one, which is the conventional plan view, and
never treats ``p_p_d`` as "up".
"""

from __future__ import annotations

import argparse
import csv
import itertools
from pathlib import Path

import matplotlib
import numpy as np

from .runlog import read_run

#: Colour cycle; distinguishable in print and for the common colour-vision
#: deficiencies, which matters because these figures go into a report.
CYCLE = ['#0072B2', '#D55E00', '#009E73', '#CC79A7',
         '#E69F00', '#56B4E9', '#8C564B', '#444444']


def _is_sweep(path: Path) -> bool:
    """A sweep summary has no ``#`` metadata and starts with ``law,N,nu``."""
    with path.open() as fh:
        for line in fh:
            if line.startswith('#'):
                continue
            return line.strip().startswith('law,N,nu')
    return False


# --------------------------------------------------------------------------- #
# Run mode
# --------------------------------------------------------------------------- #

def _label(meta: dict, path: Path) -> str:
    law = meta.get('law') or path.stem
    bits = [law]
    if meta.get('N') not in (None, ''):
        try:
            bits.append(f'N={float(meta["N"]):g}')
        except ValueError:
            pass
    if meta.get('source') == 'sitl':
        bits.append('SITL')
    return ' '.join(bits)


def plot_runs(paths: list[Path], title: str | None = None):
    """Build the five-panel comparison figure. Returns the figure."""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(15.0, 8.0), constrained_layout=True)
    gs = fig.add_gridspec(2, 3, width_ratios=[1.45, 1.0, 1.0])
    ax_xy = fig.add_subplot(gs[:, 0])
    ax_r = fig.add_subplot(gs[0, 1])
    ax_ld = fig.add_subplot(gs[0, 2])
    ax_la = fig.add_subplot(gs[1, 1])
    ax_eps = fig.add_subplot(gs[1, 2])

    a_max_seen: set[float] = set()
    capture_seen: set[float] = set()
    target_drawn = False

    for path, colour in zip(paths, itertools.cycle(CYCLE)):
        col, meta = read_run(path)
        label = _label(meta, path)
        t = col['t']

        # --- plan view (East right, North up) ----------------------------
        ax_xy.plot(col['p_p_e'], col['p_p_n'], color=colour, lw=1.6,
                   label=f'pursuer: {label}')
        ax_xy.plot(col['p_p_e'][0], col['p_p_n'][0], 'o', color=colour, ms=6)
        ax_xy.plot(col['p_p_e'][-1], col['p_p_n'][-1], 'x', color=colour,
                   ms=9, mew=2)

        # The target course is shared across runs of the same engagement, so
        # draw it once rather than stacking identical lines.
        if not target_drawn:
            ax_xy.plot(col['p_t_e'], col['p_t_n'], color='0.35', lw=1.2,
                       ls='--', label='target')
            ax_xy.plot(col['p_t_e'][0], col['p_t_n'][0], 's', color='0.35',
                       ms=6)
            target_drawn = True

        ax_r.plot(t, col['R'], color=colour, lw=1.5, label=label)
        ax_ld.plot(t, np.degrees(col['lamdot']), color=colour, lw=1.5,
                   label=label)
        ax_la.plot(t, col['latax'], color=colour, lw=1.5, label=label)
        ax_eps.plot(t, np.degrees(col['eps']), color=colour, lw=1.5,
                    label=label)

        for key, sink in (('a_max', a_max_seen), ('capture', capture_seen)):
            try:
                sink.add(float(meta[key]))
            except (KeyError, ValueError):
                pass

    # Saturation bounds: these make "pursuit is saturated, PN is not" visible
    # without reading numbers off the axis.
    for a_max in sorted(a_max_seen):
        for sign in (+1.0, -1.0):
            ax_la.axhline(sign * a_max, color='0.5', ls=':', lw=1.2)
    if a_max_seen:
        ax_la.text(0.02, 0.94, f'$a_{{max}} = {max(a_max_seen):g}$ m/s$^2$',
                   transform=ax_la.transAxes, fontsize=9, color='0.35',
                   va='top')
    for capture in sorted(capture_seen):
        ax_r.axhline(capture, color='0.5', ls=':', lw=1.2)

    ax_xy.set_xlabel('East [m]')
    ax_xy.set_ylabel('North [m]')
    ax_xy.set_title('Plan view (NED)')
    ax_xy.axis('equal')
    ax_xy.legend(fontsize=8, loc='best')

    ax_ld.axhline(0.0, color='0.5', lw=0.8)
    ax_eps.axhline(0.0, color='0.5', lw=0.8)

    for ax, ylabel, ttl in (
            (ax_r, 'R [m]', 'Range'),
            (ax_ld, r'$\dot\lambda$ [deg/s]', 'LOS rate'),
            (ax_la, 'latax [m/s$^2$]', 'Lateral acceleration (+ve right)'),
            (ax_eps, r'$\epsilon$ [deg]', r'Heading error $\lambda-\gamma$')):
        ax.set_xlabel('t [s]')
        ax.set_ylabel(ylabel)
        ax.set_title(ttl, fontsize=10)
        ax.grid(alpha=0.3)

    ax_r.legend(fontsize=8)
    if title:
        fig.suptitle(title, fontsize=13)
    return fig


# --------------------------------------------------------------------------- #
# Sweep mode
# --------------------------------------------------------------------------- #

def plot_sweep(path: Path, title: str | None = None):
    """Capturability chart: miss distance against speed ratio."""
    import matplotlib.pyplot as plt

    with path.open() as fh:
        rows = [r for r in csv.DictReader(
            line for line in fh if not line.startswith('#'))]

    groups: dict[tuple[str, str], list[dict]] = {}
    for row in rows:
        groups.setdefault((row['law'], row['N']), []).append(row)

    fig, (ax_miss, ax_peak) = plt.subplots(1, 2, figsize=(13.0, 5.0),
                                           constrained_layout=True)

    for (law, N), grp, colour in zip(groups.keys(), groups.values(),
                                     itertools.cycle(CYCLE)):
        grp = sorted(grp, key=lambda r: float(r['nu']))
        nu = [float(r['nu']) for r in grp]
        miss = [float(r['miss']) for r in grp]
        peak = [float(r['peak_latax']) for r in grp]
        hit = [r['outcome'] == 'hit' for r in grp]
        label = f'{law} N={float(N):g}' if law in {'pn', 'tpn', 'ppn', 'apn',
                                                   'zem'} else law

        ax_miss.plot(nu, miss, '-', color=colour, lw=1.5, label=label)
        ax_peak.plot(nu, peak, '-', color=colour, lw=1.5, label=label)
        # Filled marker = captured, hollow = missed. One glance tells you the
        # capturability boundary.
        for ax, ydata in ((ax_miss, miss), (ax_peak, peak)):
            for x, y, ok in zip(nu, ydata, hit):
                ax.plot(x, y, 'o', color=colour, ms=6,
                        mfc=colour if ok else 'white', mew=1.4)

    ax_miss.set_yscale('log')
    ax_miss.set_ylabel('miss distance [m]')
    ax_miss.set_title('Capturability (filled = captured)', fontsize=11)
    ax_peak.set_ylabel(r'peak $|$latax$|$ [m/s$^2$]')
    ax_peak.set_title('Peak lateral acceleration demand', fontsize=11)
    for ax in (ax_miss, ax_peak):
        ax.set_xlabel(r'speed ratio $\nu = V_T / V_P$')
        ax.axvline(1.0, color='0.6', ls=':', lw=1.2)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    if title:
        fig.suptitle(title, fontsize=13)
    return fig


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog='plot_run',
        description='Plot guidance run CSVs, or a sweep summary CSV.')
    parser.add_argument('csv', nargs='+', help='run CSV(s) or one sweep CSV')
    parser.add_argument('-o', '--out', default=None,
                        help='output PNG (default: alongside the first CSV)')
    parser.add_argument('--title', default=None)
    parser.add_argument('--dpi', type=int, default=140)
    parser.add_argument('--show', action='store_true',
                        help='open an interactive window instead of only '
                             'writing the PNG')
    args = parser.parse_args(argv)

    # Pick the backend before pyplot is imported, so this works headless.
    matplotlib.use('TkAgg' if args.show else 'Agg')
    import matplotlib.pyplot as plt

    paths = [Path(p).expanduser() for p in args.csv]
    missing = [p for p in paths if not p.is_file()]
    if missing:
        parser.error('no such file: ' + ', '.join(str(p) for p in missing))

    if _is_sweep(paths[0]):
        if len(paths) > 1:
            parser.error('sweep mode takes a single summary CSV')
        fig = plot_sweep(paths[0], args.title)
        default_out = paths[0].with_suffix('.png')
    else:
        fig = plot_runs(paths, args.title)
        stem = paths[0].stem if len(paths) == 1 else 'compare'
        default_out = paths[0].with_name(f'{stem}.png')

    out = Path(args.out).expanduser() if args.out else default_out
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=args.dpi)
    print(f'wrote {out}')
    if args.show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
