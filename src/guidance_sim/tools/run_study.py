#!/usr/bin/env python3
"""Generate the full set of figures for the AS5570 report in one pass.

Runs every law over a set of named engagements, then the capturability sweep,
and writes one comparison PNG per engagement plus the sweep chart. Offline
only -- no Gazebo, no SITL -- so the whole study takes seconds and is
reproducible from a clean checkout.

    python3 tools/run_study.py --out-dir study
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running straight from the source tree without installing.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from guidance_sim.offline import main as offline_main        # noqa: E402
from guidance_sim.offline import main_sweep                  # noqa: E402
from guidance_sim.plot_run import main as plot_main          # noqa: E402

#: Engagements mirroring config/laws.yaml, as CLI argument lists.
ENGAGEMENTS = {
    'head_on': ['--r0', '500', '--lam0', '0', '--gam-t', '180',
                '--vp', '12', '--vt', '6'],
    'crossing': ['--r0', '500', '--lam0', '30', '--gam-t', '200',
                 '--vp', '12', '--vt', '6'],
    'tail_chase': ['--r0', '300', '--lam0', '0', '--gam-t', '10',
                   '--vp', '12', '--vt', '9'],
    'fast_evader': ['--r0', '600', '--lam0', '45', '--gam-t', '200',
                    '--vp', '10', '--vt', '12'],
    'weaving': ['--r0', '500', '--lam0', '20', '--gam-t', '190',
                '--vp', '12', '--vt', '6',
                '--target', 'weave', '--at', '3', '--weave-period', '8'],
}

LAWS = ['pure_pursuit', 'deviated_pursuit', 'constant_bearing',
        'pn', 'ppn', 'apn', 'zem', 'los']

#: Laws drawn on the comparison figure. All eight is unreadable; these four
#: carry the argument (saturating pursuit vs. the PN family vs. beam riding).
PLOTTED = ['pure_pursuit', 'pn', 'ppn', 'los']


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog='run_study')
    parser.add_argument('--out-dir', default='study')
    parser.add_argument('--a-max', default='5.66',
                        help='latax limit; 5.66 = g*tan(30deg), stock '
                             'ANGLE_MAX')
    args = parser.parse_args(argv)

    out = Path(args.out_dir).expanduser()
    (out / 'runs').mkdir(parents=True, exist_ok=True)

    for name, geom in ENGAGEMENTS.items():
        print(f'\n=== {name} ' + '=' * (60 - len(name)))
        for law in LAWS:
            offline_main(['--law', law, '--a-max', args.a_max, *geom,
                          '-o', str(out / 'runs' / f'{name}_{law}.csv')])
        plot_main([str(out / 'runs' / f'{name}_{law}.csv')
                   for law in PLOTTED]
                  + ['-o', str(out / f'{name}.png'),
                     '--title', f'{name}: guidance law comparison'])

    print('\n=== capturability sweep ' + '=' * 44)
    main_sweep(['--laws', 'pure_pursuit,constant_bearing,pn,ppn',
                '--n-list', '3,5',
                '--nu-list', '0.3,0.5,0.7,0.9,1.1,1.3',
                '--a-max', args.a_max,
                '--lam0', '30', '--gam-t', '200',
                '-o', str(out / 'sweep.csv')])
    plot_main([str(out / 'sweep.csv'), '-o', str(out / 'sweep.png'),
               '--title', 'Capturability vs speed ratio'])

    print(f'\nstudy written to {out.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
