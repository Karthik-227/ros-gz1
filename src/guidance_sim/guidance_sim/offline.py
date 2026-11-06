"""Point-mass offline simulator for the guidance laws.

No ROS, no Gazebo, no MAVLink -- just the kinematics, so a law can be checked
and a parameter swept in milliseconds instead of minutes. The same
:class:`~guidance_sim.engagement.Engagement` and the same law objects run here
and in :mod:`guidance_sim.engage`, and both write the schema in
:mod:`guidance_sim.runlog`, so offline and SITL runs are directly comparable.

Vehicle model
-------------
Both vehicles are constant-speed point masses steered by lateral acceleration:
the component of the command along the velocity vector is discarded, and the
remainder turns the velocity without changing its magnitude. This is the
standard guidance-analysis model and it is what makes latax the single
quantity of interest -- a quadcopter can in fact accelerate along its
velocity, but allowing that would turn every capturability question into a
question about throttle instead of guidance.

The command is held constant across a step (zero-order hold at ``1/dt``, like
a real autopilot update), and the velocity is then rotated exactly through
``|a_perp| * dt / V`` about ``v x a_perp`` via Rodrigues' formula, so speed is
conserved to machine precision rather than drifting the way an Euler update
would. Position advances on the trapezoidal average of the velocities, which
is second-order accurate over the arc.
"""

from __future__ import annotations

import argparse
import itertools
import math
from dataclasses import dataclass, field
from dataclasses import fields as dc_fields
from pathlib import Path

import numpy as np

from .engagement import Engagement, right_normal, unit
from .guidance_laws import LAWS, make_law
from .runlog import RunLogger, sample


# --------------------------------------------------------------------------- #
# Kinematics
# --------------------------------------------------------------------------- #

def lateral(a: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Component of ``a`` perpendicular to ``v`` -- the part that steers."""
    v_hat = unit(v)
    if not np.any(v_hat):
        return np.asarray(a, dtype=float)
    return a - (a @ v_hat) * v_hat


def turn(v: np.ndarray, a_perp: np.ndarray, dt: float) -> np.ndarray:
    """Rotate ``v`` under lateral acceleration ``a_perp`` for ``dt``.

    Exact for a constant ``a_perp`` over the step, and speed-preserving by
    construction (Rodrigues about an axis perpendicular to ``v``).
    """
    V = float(np.linalg.norm(v))
    mag = float(np.linalg.norm(a_perp))
    if V < 1e-9 or mag < 1e-12:
        return v
    axis = np.cross(v, a_perp)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:            # command parallel to velocity: no turn
        return v
    axis = axis / n
    ang = mag * dt / V
    return v * math.cos(ang) + np.cross(axis, v) * math.sin(ang)


# --------------------------------------------------------------------------- #
# Target manoeuvres
# --------------------------------------------------------------------------- #

@dataclass
class Target:
    """Target motion model. ``kind`` selects the manoeuvre.

    ``straight``  constant velocity -- the textbook non-manoeuvring target.
    ``turn``      constant lateral acceleration, a steady circle.
    ``weave``     sinusoidal lateral acceleration of period ``period``; the
                  hardest classical case, and the one that separates APN from
                  plain PN.
    """

    kind: str = 'straight'
    accel: float = 0.0               # lateral acceleration amplitude, m/s^2
    period: float = 10.0             # weave period, s

    def accel_at(self, t: float, v_t: np.ndarray) -> np.ndarray:
        """Target lateral acceleration in NED at time ``t``."""
        if self.kind == 'straight' or self.accel == 0.0:
            return np.zeros(3)
        if self.kind == 'turn':
            amp = self.accel
        elif self.kind == 'weave':
            amp = self.accel * math.sin(2.0 * math.pi * t / self.period)
        else:
            raise ValueError(f'unknown target motion {self.kind!r}')
        return amp * right_normal(v_t)


# --------------------------------------------------------------------------- #
# Initial conditions
# --------------------------------------------------------------------------- #

@dataclass
class InitialConditions:
    """Engagement set-up, in the NED conventions of :mod:`.engagement`.

    The pursuer starts at the origin at altitude ``alt``; the target is placed
    at range ``R0`` on bearing ``lam0`` from it. Headings are bearings from
    North, positive towards East.
    """

    R0: float = 500.0
    lam0: float = math.radians(0.0)
    gam0: float = math.radians(0.0)
    gam_t: float = math.radians(180.0)
    Vp: float = 12.0
    Vt: float = 6.0
    alt: float = 50.0
    lead_start: bool = False         # start on the collision triangle

    def states(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        down = -abs(self.alt)
        p_p = np.array([0.0, 0.0, down])
        p_t = p_p + self.R0 * np.array([math.cos(self.lam0),
                                        math.sin(self.lam0), 0.0])
        v_t = self.Vt * np.array([math.cos(self.gam_t),
                                  math.sin(self.gam_t), 0.0])
        gam0 = self.gam0
        if self.lead_start:
            probe = Engagement.from_states(
                p_p, self.Vp * np.array([math.cos(gam0), math.sin(gam0), 0.0]),
                p_t, v_t)
            lead = probe.collision_heading()
            if lead is not None:
                gam0 = lead
        v_p = self.Vp * np.array([math.cos(gam0), math.sin(gam0), 0.0])
        return p_p, v_p, p_t, v_t


# --------------------------------------------------------------------------- #
# The run
# --------------------------------------------------------------------------- #

@dataclass
class RunResult:
    """Outcome of one engagement, plus the full time history."""

    outcome: str = 'timeout'         # hit | miss | timeout
    miss: float = float('inf')       # refined closest approach, m
    r_min: float = float('inf')      # smallest sampled range, m
    t_f: float = 0.0                 # time at termination, s
    peak_latax: float = 0.0
    mean_abs_latax: float = 0.0
    total_dv: float = 0.0            # integral of |a| dt -- control effort
    logger: RunLogger = field(default_factory=RunLogger)

    def summary(self) -> dict:
        return {
            'outcome': self.outcome,
            'miss': self.miss,
            'r_min': self.r_min,
            't_f': self.t_f,
            'peak_latax': self.peak_latax,
            'mean_abs_latax': self.mean_abs_latax,
            'total_dv': self.total_dv,
        }


def _closest_approach(eng: Engagement) -> float:
    """Perpendicular miss if both vehicles froze their velocities now.

    Used to refine the sampled miss distance: at the step where ``Rdot``
    changes sign, the true closest approach lies between samples, and this is
    its exact constant-velocity value.
    """
    vr = float(np.linalg.norm(eng.v_rel))
    if vr < 1e-9:
        return eng.R
    return float(np.linalg.norm(np.cross(eng.r, eng.v_rel)) / vr)


def simulate(law, ic: InitialConditions | None = None,
             target: Target | None = None,
             dt: float = 0.01, t_max: float = 120.0,
             capture: float = 1.0, meta: dict | None = None) -> RunResult:
    """Fly one engagement and return the result with its log."""
    ic = ic or InitialConditions()
    target = target or Target()
    law.reset()

    p_p, v_p, p_t, v_t = ic.states()
    res = RunResult(logger=RunLogger(meta=meta))

    n_steps = int(round(t_max / dt))
    t = 0.0
    latax_sum = 0.0
    prev_rdot = None
    diverging = False

    for _ in range(n_steps + 1):
        a_t = target.accel_at(t, v_t)
        eng = Engagement.from_states(p_p, v_p, p_t, v_t, a_t)
        a_cmd = law(eng, dt)

        res.logger.add(sample(t, eng, a_cmd, law.extra_log()))

        latax = abs(eng.latax_of(a_cmd))
        res.peak_latax = max(res.peak_latax, latax)
        latax_sum += latax
        res.total_dv += float(np.linalg.norm(a_cmd)) * dt
        res.r_min = min(res.r_min, eng.R)
        res.t_f = t

        # --- termination -------------------------------------------------
        if eng.R <= capture:
            res.outcome = 'hit'
            res.miss = min(res.miss, eng.R)
            break
        if prev_rdot is not None and prev_rdot < 0.0 <= eng.Rdot:
            # Closest approach just passed; refine it from this sample.
            res.miss = min(res.miss, _closest_approach(eng), eng.R)
            res.outcome = 'hit' if res.miss <= capture else 'miss'
            break
        if eng.R > 3.0 * ic.R0 and eng.Rdot > 0.0:
            diverging = True
            break
        prev_rdot = eng.Rdot

        # --- advance -----------------------------------------------------
        a_p_perp = lateral(a_cmd, v_p)
        v_p_new = turn(v_p, a_p_perp, dt)
        p_p = p_p + 0.5 * (v_p + v_p_new) * dt
        v_p = v_p_new

        a_t_perp = lateral(a_t, v_t)
        v_t_new = turn(v_t, a_t_perp, dt)
        p_t = p_t + 0.5 * (v_t + v_t_new) * dt
        v_t = v_t_new

        t += dt

    if res.outcome == 'timeout' and diverging:
        res.outcome = 'miss'
    if not math.isfinite(res.miss):
        res.miss = res.r_min
    n = len(res.logger.rows)
    res.mean_abs_latax = latax_sum / n if n else 0.0
    res.logger.meta.update({k: v for k, v in res.summary().items()})
    return res


# --------------------------------------------------------------------------- #
# Command line
# --------------------------------------------------------------------------- #

def _law_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group('guidance law')
    g.add_argument('--law', default='pn', choices=sorted(LAWS),
                   help='guidance law to fly (default: pn)')
    g.add_argument('-N', '--N', type=float, default=None,
                   help='navigation ratio for the PN family (default 3)')
    g.add_argument('-k', '--k', type=float, default=None,
                   help='heading gain for the pursuit family (default 2)')
    g.add_argument('--delta', type=float, default=None,
                   help='deviated-pursuit lead angle, deg')
    g.add_argument('--kp', type=float, default=None, help='CLOS position gain')
    g.add_argument('--kd', type=float, default=None, help='CLOS rate gain')
    # ArduCopter's ANGLE_MAX defaults to 30 deg, so the airframe can only
    # deliver g*tan(30 deg) = 5.66 m/s^2. Offline and SITL must use the same
    # bound or the comparison is meaningless.
    g.add_argument('--a-max', type=float, default=5.66,
                   help='latax limit, m/s^2 (default 5.66 = g*tan(30deg))')


def _geometry_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group('engagement geometry')
    g.add_argument('--r0', type=float, default=500.0, help='initial range, m')
    g.add_argument('--lam0', type=float, default=0.0,
                   help='initial LOS bearing, deg from North')
    g.add_argument('--gam0', type=float, default=0.0,
                   help='initial pursuer heading, deg from North')
    g.add_argument('--gam-t', type=float, default=180.0,
                   help='target heading, deg from North (default head-on)')
    g.add_argument('--vp', type=float, default=12.0, help='pursuer speed, m/s')
    g.add_argument('--vt', type=float, default=6.0, help='target speed, m/s')
    g.add_argument('--alt', type=float, default=50.0, help='altitude, m')
    g.add_argument('--lead-start', action='store_true',
                   help='start the pursuer on the collision triangle')

    m = parser.add_argument_group('target manoeuvre')
    m.add_argument('--target', default='straight',
                   choices=['straight', 'turn', 'weave'])
    m.add_argument('--at', type=float, default=0.0,
                   help='target lateral acceleration amplitude, m/s^2')
    m.add_argument('--weave-period', type=float, default=10.0,
                   help='weave period, s')

    s = parser.add_argument_group('solver')
    s.add_argument('--dt', type=float, default=0.01, help='time step, s')
    s.add_argument('--t-max', type=float, default=120.0, help='time limit, s')
    s.add_argument('--capture', type=float, default=1.0,
                   help='capture radius, m')


def _build(args) -> tuple[object, InitialConditions, Target, dict]:
    kwargs = dict(N=args.N, k=args.k, a_max=args.a_max,
                  kp=args.kp, kd=args.kd)
    if args.delta is not None:
        kwargs['delta'] = math.radians(args.delta)
    law = make_law(args.law, **kwargs)

    ic = InitialConditions(
        R0=args.r0, lam0=math.radians(args.lam0), gam0=math.radians(args.gam0),
        gam_t=math.radians(args.gam_t), Vp=args.vp, Vt=args.vt, alt=args.alt,
        lead_start=args.lead_start)

    # CLOS is station referenced; the natural station is the pursuer's launch
    # point, so put it there rather than leaving it at the NED origin (which
    # would read the pursuer's altitude as a cross-beam error).
    if hasattr(law, 'station'):
        law.station = ic.states()[0].copy()

    target = Target(kind=args.target, accel=args.at, period=args.weave_period)
    meta = {
        'law': law.name, 'a_max': law.a_max,
        'N': getattr(law, 'N', ''), 'k': getattr(law, 'k', ''),
        'Vp': ic.Vp, 'Vt': ic.Vt, 'nu': ic.Vt / ic.Vp if ic.Vp else '',
        'R0': ic.R0, 'lam0_deg': args.lam0, 'gam0_deg': args.gam0,
        'gam_t_deg': args.gam_t, 'alt': ic.alt,
        'target': f'{target.kind}@{target.accel}',
        'dt': args.dt, 'capture': args.capture, 'source': 'offline',
    }
    return law, ic, target, meta


def main(argv=None) -> int:
    """Fly a single engagement and write its CSV."""
    parser = argparse.ArgumentParser(
        prog='offline',
        description='Point-mass simulation of one guidance engagement.')
    _law_args(parser)
    _geometry_args(parser)
    parser.add_argument('-o', '--out', default='runs/run.csv',
                        help='output CSV path (default runs/run.csv)')
    args = parser.parse_args(argv)

    law, ic, target, meta = _build(args)
    res = simulate(law, ic, target, dt=args.dt, t_max=args.t_max,
                   capture=args.capture, meta=meta)
    path = res.logger.write(args.out)

    print(f'{law.name:16} {res.outcome:8} '
          f'miss={res.miss:7.2f} m  t_f={res.t_f:6.2f} s  '
          f'peak|latax|={res.peak_latax:5.2f} m/s^2  '
          f'mean={res.mean_abs_latax:5.2f}  dv={res.total_dv:6.1f} m/s')
    print(f'wrote {path} ({len(res.logger.rows)} rows)')
    return 0


def _floats(text: str) -> list[float]:
    return [float(x) for x in text.replace(' ', '').split(',') if x]


def main_sweep(argv=None) -> int:
    """Sweep law x navigation-ratio x speed-ratio and summarise each run.

    This is the capturability study: for each combination it records whether
    the pursuer captured, the miss distance, and the latax it needed, which is
    what shows PN capturing a faster target where pure pursuit cannot.
    """
    parser = argparse.ArgumentParser(
        prog='sweep',
        description='Sweep guidance parameters and summarise the outcomes.')
    _geometry_args(parser)
    parser.add_argument('--laws', default='pure_pursuit,pn,ppn,apn,zem',
                        help='comma-separated laws to sweep')
    parser.add_argument('--n-list', default='3,4,5',
                        help='comma-separated navigation ratios')
    parser.add_argument('--nu-list', default='0.3,0.5,0.7,0.9,1.1',
                        help='comma-separated speed ratios Vt/Vp')
    parser.add_argument('--a-max', type=float, default=5.66,
                        help='latax limit, m/s^2 (default 5.66)')
    parser.add_argument('-o', '--out', default='runs/sweep.csv',
                        help='output summary CSV')
    args = parser.parse_args(argv)

    laws = [s for s in args.laws.replace(' ', '').split(',') if s]
    n_list = _floats(args.n_list)
    nu_list = _floats(args.nu_list)

    rows: list[dict] = []
    print(f'{"law":18} {"N":>4} {"nu":>5} {"outcome":8} {"miss":>8} '
          f'{"t_f":>7} {"peak":>6} {"dv":>7}')
    print('-' * 70)

    has_N = {name for name in laws
             if 'N' in {f.name for f in dc_fields(LAWS[name])}}

    for law_name, N, nu in itertools.product(laws, n_list, nu_list):
        # Pursuit and CLOS laws have no navigation ratio, so sweeping N would
        # just re-fly an identical engagement; run those once.
        if law_name not in has_N and N != n_list[0]:
            continue
        target = Target(kind=args.target, accel=args.at,
                        period=args.weave_period)
        ic = InitialConditions(
            R0=args.r0, lam0=math.radians(args.lam0),
            gam0=math.radians(args.gam0), gam_t=math.radians(args.gam_t),
            Vp=args.vp, Vt=nu * args.vp, alt=args.alt,
            lead_start=args.lead_start)
        law = make_law(law_name, N=N, a_max=args.a_max)
        if hasattr(law, 'station'):
            law.station = ic.states()[0].copy()

        res = simulate(law, ic, target, dt=args.dt, t_max=args.t_max,
                       capture=args.capture)
        row = {'law': law_name, 'N': N, 'nu': nu, **res.summary()}
        rows.append(row)
        print(f'{law_name:18} {N:4.1f} {nu:5.2f} {res.outcome:8} '
              f'{res.miss:8.2f} {res.t_f:7.2f} {res.peak_latax:6.2f} '
              f'{res.total_dv:7.1f}')

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    import csv as _csv
    with out.open('w', newline='') as fh:
        writer = _csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'\nwrote {out} ({len(rows)} runs)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
