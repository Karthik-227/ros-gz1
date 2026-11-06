"""Fly a guidance engagement against two live ArduPilot SITL instances.

This is the Gazebo-side twin of :mod:`guidance_sim.offline`. It builds the
same :class:`~guidance_sim.engagement.Engagement` each tick, evaluates the same
law object, and writes the same CSV schema with ``source: sitl`` in the
metadata, so a flown run and a simulated one overlay directly in
:mod:`guidance_sim.plot_run`.

Loop structure
--------------
The pursuer is commanded with raw acceleration setpoints in GUIDED (see
:mod:`guidance_sim.mavlink_io` for why the type mask is what it is). The
target is flown on a prescribed course with velocity setpoints, so its
acceleration is *known analytically* rather than differentiated from noisy
telemetry -- which is what lets APN's feed-forward term be exercised honestly
rather than against a numerically differentiated estimate.

Vertical axis
-------------
``set_accel_NED_mss`` also takes a Down acceleration, and commanding zero there
requests zero *vertical acceleration*, not altitude hold -- any vertical
velocity simply keeps integrating. Classical latax is a horizontal-plane
quantity, so by default the guidance law's Down component is discarded and the
Down channel is driven by a PD altitude hold instead. ``--vertical law`` hands
the channel back to the guidance law for genuinely three-dimensional work; the
two must never both write it.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from .engagement import Engagement
from .guidance_laws import LAWS, make_law
from .mavlink_io import VehicleError, connect, heading_to_velocity
from .offline import Target, lateral, turn
from .runlog import RunLogger, sample


def altitude_hold(z_target: float, z_now: float, v_down: float,
                  kp: float = 1.0, kd: float = 2.0,
                  limit: float = 3.0) -> float:
    """PD on the Down axis, returning a Down acceleration in m/s^2.

    Signs follow NED: ``z`` is positive down, so being *below* the target
    altitude means ``z_now > z_target`` and the command comes out negative,
    i.e. upward. Clipped so a vertical transient can never eat the whole
    acceleration budget that the guidance law needs horizontally.
    """
    a_d = kp * (z_target - z_now) - kd * v_down
    return float(np.clip(a_d, -limit, limit))


def budget(a_cmd: np.ndarray, a_down: float, a_max: float) -> np.ndarray:
    """Fit the horizontal command and the vertical term inside one ``a_max``.

    The guidance law is clipped to ``a_max`` on its own and the altitude hold
    is clipped on its own, but their resultant is not: a saturated law plus a
    full vertical correction asks for ``hypot(5.66, 3.0) = 6.4`` m/s^2, which
    at ``ANGLE_MAX = 30 deg`` is a 33 degree lean the copter cannot make.
    ArduCopter would clip that itself, but it would do so by shrinking the
    whole vector, silently eating into the guidance command.

    Altitude keeps its share -- falling out of the sky is worse than missing
    -- and whatever acceleration is left over goes to the horizontal plane.
    """
    a_down = float(np.clip(a_down, -a_max, a_max))
    h_allow = math.sqrt(max(a_max**2 - a_down**2, 0.0))
    h = np.array([a_cmd[0], a_cmd[1], 0.0])
    h_norm = float(np.linalg.norm(h))
    if h_norm > h_allow and h_norm > 1e-12:
        h = h * (h_allow / h_norm)
    return np.array([h[0], h[1], a_down])


def _closest_approach(eng: Engagement) -> float:
    """Perpendicular miss if both vehicles froze their velocities now."""
    vr = float(np.linalg.norm(eng.v_rel))
    if vr < 1e-9:
        return eng.R
    return float(np.linalg.norm(np.cross(eng.r, eng.v_rel)) / vr)


def target_setpoint(v_t_cmd: np.ndarray, z_now: float, z_target: float,
                    kp: float = 0.5, limit: float = 2.0) -> np.ndarray:
    """Target's horizontal course plus a vertical trim, as a velocity setpoint.

    The engagement maths uses the purely horizontal ``v_t_cmd``, matching the
    offline target model; this trims only the vertical channel of what is
    actually *sent*, so the target holds the engagement altitude instead of
    drifting off it.

    Without this the target keeps whatever altitude it happened to have when
    the run started, because a ``vz = 0`` velocity setpoint holds the current
    height rather than a commanded one. A flown run with the target 33 m below
    the pursuer cannot close to less than 33 m however good the law is -- the
    engagement is planar in name only.
    """
    vz = float(np.clip(kp * (z_target - z_now), -limit, limit))
    return np.array([v_t_cmd[0], v_t_cmd[1], vz])


def speed_hold(v_p: np.ndarray, v_cmd: float, gam0: float,
               gain: float = 0.5, limit: float = 2.0) -> np.ndarray:
    """Along-track acceleration holding the pursuer's horizontal speed.

    The offline model keeps ``|v_p|`` constant by assumption; a real copter
    does not, so the live path has to hold it actively. This is the throttle
    half of the job -- the guidance law steers, this keeps the speed the law
    assumes it has.

    Below 0.1 m/s there is no velocity direction to push along (and
    ``v / |v|`` would amplify noise), so the push goes along ``gam0`` instead
    to get the vehicle moving at all.
    """
    v_h = np.array([v_p[0], v_p[1], 0.0])
    speed = float(np.linalg.norm(v_h))
    if speed > 0.1:
        direction = v_h / speed
    else:
        direction = np.array([math.cos(gam0), math.sin(gam0), 0.0])
    a = gain * (v_cmd - speed) * direction
    n = float(np.linalg.norm(a))
    if n > limit > 0.0:
        a = a * (limit / n)
    return a


def blend(a_lat: np.ndarray, a_along: np.ndarray, limit: float) -> np.ndarray:
    """Fit the steering and speed-hold commands inside one horizontal limit.

    The airframe can only deliver ``g*tan(ANGLE_MAX)`` horizontally in total,
    however the command is decomposed. Guidance gets first call on it -- in a
    saturated turn, turning matters more than holding speed -- and the
    speed-hold term takes whatever is left. Same priority idea as
    :func:`budget`, one level down.
    """
    lat = np.array([a_lat[0], a_lat[1], 0.0])
    lat_norm = float(np.linalg.norm(lat))
    if lat_norm > limit and lat_norm > 1e-12:
        return lat * (limit / lat_norm)
    spare = math.sqrt(max(limit**2 - lat_norm**2, 0.0))
    along = np.array([a_along[0], a_along[1], 0.0])
    along_norm = float(np.linalg.norm(along))
    if along_norm > spare and along_norm > 1e-12:
        along = along * (spare / along_norm)
    # The two terms are not orthogonal in general -- PN commands perpendicular
    # to the LOS, not to the velocity, so its command has an along-track
    # component of its own. Clip the resultant rather than trusting
    # sqrt(lat^2 + along^2) to stay inside the limit.
    total = lat + along
    total_norm = float(np.linalg.norm(total))
    if total_norm > limit and total_norm > 1e-12:
        total = total * (limit / total_norm)
    return total


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog='engage',
        description='Fly a guidance engagement on two live SITL copters.')

    link = parser.add_argument_group('links')
    link.add_argument('--pursuer-url', default='udp:127.0.0.1:14550')
    link.add_argument('--target-url', default='udp:127.0.0.1:14560')

    law_g = parser.add_argument_group('guidance law')
    law_g.add_argument('--law', default='pn', choices=sorted(LAWS))
    law_g.add_argument('-N', '--N', type=float, default=None)
    law_g.add_argument('-k', '--k', type=float, default=None)
    law_g.add_argument('--delta', type=float, default=None,
                       help='deviated-pursuit lead angle, deg')
    law_g.add_argument('--kp', type=float, default=None, help='CLOS gain')
    law_g.add_argument('--kd', type=float, default=None, help='CLOS rate gain')
    # ANGLE_MAX defaults to 30 deg, so g*tan(30 deg) = 5.66 m/s^2 is all the
    # airframe can actually deliver. Keep this consistent with offline runs.
    law_g.add_argument('--a-max', type=float, default=5.66,
                       help='latax limit, m/s^2 (default 5.66 = g*tan(30deg))')

    pur = parser.add_argument_group('pursuer')
    # The offline point-mass model starts the pursuer AT Vp and holds its speed
    # constant by assumption (offline.lateral() discards the along-track part
    # of every command). A SITL copter starts hovering at zero, and no
    # classical guidance law can fix that: PN, pursuit and CLOS all only steer
    # an existing velocity vector. Without a cruise speed, |v_p| ~ 0 makes
    # right_normal(v_p) meaningless, gamdot = latax/|v| explodes, and the
    # copter spins on the spot instead of flying an engagement.
    pur.add_argument('--vp', type=float, default=12.0,
                     help='pursuer cruise speed, m/s (default 12, matching '
                          'the offline default)')
    pur.add_argument('--gam0', type=float, default=None,
                     help='initial pursuer heading, deg from North '
                          '(default: straight at the target)')
    pur.add_argument('--boost', type=float, default=15.0,
                     help='seconds allowed to reach cruise speed before '
                          'guidance starts (default 15)')
    pur.add_argument('--speed-gain', type=float, default=0.5,
                     help='along-track gain holding |v_p| at --vp during '
                          'guidance, 1/s (default 0.5)')
    pur.add_argument('--a-speed', type=float, default=2.0,
                     help='cap on the along-track speed-hold command, m/s^2 '
                          '(default 2); it yields to the guidance law')

    run = parser.add_argument_group('run')
    run.add_argument('--alt', type=float, default=50.0)
    run.add_argument('--takeoff', action='store_true',
                     help='arm and take off both vehicles first')
    run.add_argument('--rate', type=float, default=50.0,
                     help='guidance loop rate, Hz (default 50)')
    run.add_argument('--t-max', type=float, default=180.0)
    run.add_argument('--capture', type=float, default=2.0,
                     help='capture radius, m (default 2)')
    run.add_argument('--vertical', choices=['hold', 'law'], default='hold',
                     help="'hold' keeps altitude with a PD and discards the "
                          "law's Down component (default); 'law' hands the "
                          'Down channel to the guidance law')
    run.add_argument('--settle', type=float, default=3.0,
                     help='seconds to fly the target before guidance starts')
    run.add_argument('--on-finish', choices=['loiter', 'land', 'nothing'],
                     default='loiter')
    run.add_argument('--status', type=float, default=1.0,
                     help='seconds between live status lines; 0 to silence '
                          '(default 1). Printing every tick at 50 Hz would '
                          'itself slow the loop down')
    run.add_argument('-o', '--out', default='runs/sitl_run.csv')

    tgt = parser.add_argument_group('target course')
    tgt.add_argument('--vt', type=float, default=6.0, help='target speed, m/s')
    tgt.add_argument('--gam-t', type=float, default=180.0,
                     help='target heading, deg from North')
    tgt.add_argument('--target', default='straight',
                     choices=['straight', 'turn', 'weave'])
    tgt.add_argument('--at', type=float, default=0.0,
                     help='target lateral acceleration amplitude, m/s^2')
    tgt.add_argument('--weave-period', type=float, default=10.0)

    args = parser.parse_args(argv)

    kwargs = dict(N=args.N, k=args.k, a_max=args.a_max,
                  kp=args.kp, kd=args.kd)
    if args.delta is not None:
        kwargs['delta'] = math.radians(args.delta)
    law = make_law(args.law, **kwargs)
    target = Target(kind=args.target, accel=args.at,
                    period=args.weave_period)

    pursuer = tgt_veh = logger = None
    gam0 = 0.0 if args.gam0 is None else math.radians(args.gam0)
    t0 = time.monotonic()
    outcome, miss, r_min = 'timeout', float('inf'), float('inf')
    peak_latax, latax_sum, total_dv, n = 0.0, 0.0, 0.0, 0
    meta: dict = {}
    try:
        pursuer = connect(args.pursuer_url, 'pursuer', rate_hz=args.rate)
        tgt_veh = connect(args.target_url, 'target', rate_hz=args.rate)

        if args.takeoff:
            for veh in (pursuer, tgt_veh):
                veh.set_mode('GUIDED')
                veh.arm()
            for veh in (pursuer, tgt_veh):
                veh.takeoff(args.alt)
        else:
            for veh in (pursuer, tgt_veh):
                veh.set_mode('GUIDED')

        # CLOS is station referenced; the natural station is the pursuer's
        # position at the start of the run.
        p_p0, _ = pursuer.state()
        if hasattr(law, 'station'):
            law.station = p_p0.copy()
        law.reset()

        z_target = -abs(args.alt)
        v_t_cmd = heading_to_velocity(args.vt, math.radians(args.gam_t))

        meta.update({
            'law': law.name, 'a_max': law.a_max,
            'N': getattr(law, 'N', ''), 'k': getattr(law, 'k', ''),
            'Vt_cmd': args.vt, 'gam_t_deg': args.gam_t, 'alt': args.alt,
            'target': f'{target.kind}@{target.accel}',
            'rate_hz': args.rate, 'capture': args.capture,
            'vertical': args.vertical, 'source': 'sitl',
        })
        logger = RunLogger(meta=meta)

        # --- fly the target onto its course before guidance begins -------
        period = 1.0 / args.rate
        print(f'[run] settling target for {args.settle:.1f} s')
        settle_end = time.monotonic() + args.settle
        while time.monotonic() < settle_end:
            tgt_veh.poll()
            pursuer.poll()
            p_t, _ = tgt_veh.state()
            tgt_veh.send_velocity_ned(
                target_setpoint(v_t_cmd, p_t[2], z_target))
            p_p, v_p = pursuer.state()
            pursuer.send_accel_ned(
                [0.0, 0.0, altitude_hold(z_target, p_p[2], v_p[2])])
            time.sleep(period)

        # --- accelerate the pursuer to cruise speed ----------------------
        # Guidance cannot start from a hover: see the --vp help text. Fly the
        # pursuer up to speed on velocity setpoints first, pointing it along
        # --gam0 (default: straight at the target), which is the live
        # equivalent of the offline InitialConditions starting at Vp on
        # bearing gam0.
        p_p, _ = pursuer.state()
        p_t, _ = tgt_veh.state()
        dz = abs(p_t[2] - p_p[2])
        if dz > 2.0:
            # Reported, not silently corrected: if the two are tens of metres
            # apart vertically the engagement is not the planar one the law
            # and the offline comparison assume, and the miss distance can
            # never beat the height difference.
            print(f'warning: {dz:.1f} m of vertical separation '
                  f'(pursuer {-p_p[2]:.1f} m, target {-p_t[2]:.1f} m). '
                  f'The target is being trimmed to {args.alt:g} m, but give '
                  f'it a few seconds to get there before trusting R.')
        if args.gam0 is None:
            gam0 = math.atan2(p_t[1] - p_p[1], p_t[0] - p_p[0])
            print(f'[run] aiming at the target, gam0 = '
                  f'{math.degrees(gam0):.1f} deg from North')
        else:
            gam0 = math.radians(args.gam0)
        v_p_cmd = heading_to_velocity(args.vp, gam0)

        print(f'[run] accelerating pursuer to {args.vp:g} m/s '
              f'(up to {args.boost:g} s)')
        boost_end = time.monotonic() + args.boost
        while time.monotonic() < boost_end:
            now = time.monotonic()
            pursuer.poll()
            tgt_veh.poll()
            pursuer.send_velocity_ned(v_p_cmd)
            p_t, _ = tgt_veh.state()
            tgt_veh.send_velocity_ned(
                target_setpoint(v_t_cmd, p_t[2], z_target))
            _, v_p = pursuer.state()
            if float(np.linalg.norm(v_p[:2])) >= 0.95 * args.vp:
                break
            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

        _, v_p = pursuer.state()
        reached = float(np.linalg.norm(v_p[:2]))
        print(f'[run] pursuer at {reached:.2f} m/s')
        if reached < 0.5 * args.vp:
            # Starting guidance from here would just reproduce the spin-on-the
            # -spot failure, so say so rather than logging 30 s of nonsense.
            print(f'warning: pursuer only reached {reached:.2f} m/s of the '
                  f'{args.vp:g} m/s requested. Guidance needs a velocity '
                  f'vector to steer; check WPNAV_SPEED and ANGLE_MAX, or '
                  f'raise --boost.')

        # --- guidance loop -----------------------------------------------
        print(f'[run] {law.name} engaging at {args.rate:g} Hz')
        t0 = time.monotonic()
        t_prev = t0
        prev_rdot = None
        next_status = 0.0
        if args.status > 0:
            print('     t      R     Rdot      Vp     Vt   lamdot    latax'
                  '   alt_p  alt_t')

        while True:
            now = time.monotonic()
            t = now - t0
            dt = now - t_prev
            t_prev = now
            if t > args.t_max:
                break

            pursuer.poll()
            tgt_veh.poll()
            p_p, v_p = pursuer.state()
            p_t, v_t = tgt_veh.state()

            a_t = target.accel_at(t, v_t_cmd)
            eng = Engagement.from_states(p_p, v_p, p_t, v_t, a_t)
            a_cmd = np.asarray(law(eng, max(dt, 1e-3)), dtype=float)

            # Hold the cruise speed the law assumes it has. Guidance keeps
            # priority inside the horizontal envelope; speed-hold gets the
            # remainder.
            a_cmd = blend(a_cmd,
                          speed_hold(v_p, args.vp, gam0,
                                     gain=args.speed_gain,
                                     limit=args.a_speed),
                          law.a_max)

            if args.vertical == 'hold':
                a_cmd = budget(a_cmd,
                               altitude_hold(z_target, p_p[2], v_p[2]),
                               law.a_max)
            pursuer.send_accel_ned(a_cmd)

            # Advance the target's commanded course the same way the offline
            # integrator does, so the two simulators fly identical profiles.
            v_t_cmd = turn(v_t_cmd, lateral(a_t, v_t_cmd), max(dt, 1e-3))
            tgt_veh.send_velocity_ned(
                target_setpoint(v_t_cmd, p_t[2], z_target))

            logger.add(sample(t, eng, a_cmd, law.extra_log()))
            latax = abs(eng.latax_of(a_cmd))
            peak_latax = max(peak_latax, latax)
            latax_sum += latax
            total_dv += float(np.linalg.norm(a_cmd)) * dt
            r_min = min(r_min, eng.R)
            n += 1

            # Live status. Rdot < 0 means closing; Vp is the pursuer's full 3-D
            # speed, so a climbing copter reads fast while going nowhere
            # horizontally -- which is exactly how the stationary-pursuer
            # failure used to hide.
            if args.status > 0 and t >= next_status:
                print(f'{t:6.1f} {eng.R:7.1f} {eng.Rdot:7.2f} {eng.Vp:7.2f} '
                      f'{eng.Vt:6.2f} {eng.lamdot:8.4f} '
                      f'{eng.latax_of(a_cmd):8.3f} '
                      f'{-p_p[2]:6.1f} {-p_t[2]:6.1f}')
                next_status += args.status

            if eng.R <= args.capture:
                outcome, miss = 'hit', min(miss, eng.R)
                break
            if prev_rdot is not None and prev_rdot < 0.0 <= eng.Rdot:
                miss = min(miss, _closest_approach(eng), eng.R)
                outcome = 'hit' if miss <= args.capture else 'miss'
                break
            prev_rdot = eng.Rdot

            sleep = period - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print('\n[run] interrupted')
        outcome = 'interrupted'
    except VehicleError as exc:
        print(f'error: {exc}')
        return 1
    finally:
        # Write whatever was flown, including a part-finished run: an
        # interrupted engagement still carries the geometry up to the abort,
        # and throwing that away because of a Ctrl-C is indefensible.
        if logger is not None and logger.rows:
            if not math.isfinite(miss):
                miss = r_min
            t_f = time.monotonic() - t0
            # RunLogger copies the meta dict it is given (runlog.py:
            # 'self.meta = dict(meta or {})'), so updating the local `meta`
            # here would never reach the file -- every SITL CSV written before
            # this was fixed is missing its outcome, miss and t_f comments.
            # Write through the logger's own dict.
            logger.meta.update({'outcome': outcome, 'miss': miss,
                                'r_min': r_min, 't_f': t_f,
                                'peak_latax': peak_latax,
                                'mean_abs_latax': latax_sum / n if n else 0.0,
                                'total_dv': total_dv,
                                'vp_cmd': args.vp,
                                'gam0_deg': math.degrees(gam0)})
            path = logger.write(args.out)
            print(f'{law.name:16} {outcome:11} miss={miss:7.2f} m  '
                  f't_f={t_f:6.2f} s  peak|latax|={peak_latax:5.2f} m/s^2  '
                  f'mean={latax_sum / n if n else 0:5.2f}  '
                  f'dv={total_dv:6.1f} m/s')
            print(f'wrote {path} ({len(logger.rows)} rows, '
                  f'{n / t_f if t_f else 0:.1f} Hz achieved)')

        # Stop commanding before anything else: a stale acceleration setpoint
        # is the one thing that must not outlive this process. ArduCopter
        # would zero it after GUID_TIMEOUT anyway, but do not rely on that.
        for veh, role in ((pursuer, 'pursuer'), (tgt_veh, 'target')):
            if veh is None:
                continue
            try:
                if args.on_finish == 'loiter':
                    veh.set_mode('LOITER')
                elif args.on_finish == 'land':
                    veh.set_mode('LAND')
            except VehicleError as exc:
                print(f'warning: {role} cleanup: {exc}')
            veh.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
