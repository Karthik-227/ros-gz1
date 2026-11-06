#!/usr/bin/env python3
"""
Planar guidance simulation for the AS5570 computer assignments.

Runs one guidance scheme at SEVERAL speed ratios in one go, then lets you plot
any variable against any other -- one panel per speed ratio, or all of them
overlaid on a single panel.

Notation (pursuer = P, target = T; no "missile"):
    xp, yp, xt, yt      positions [m]
    vp, vt              speeds [m/s]
    alpha_p, alpha_t    flight path angles (headings)
    theta               line-of-sight angle
    theta_dot           LOS rate
    ap, at              lateral accelerations
    nu = vp / vt        speed ratio
    psi = alpha_t - theta
"""

import math
import matplotlib

# The Qt backend aborts on some GNOME setups with
#   GLib-GIO-ERROR: Settings schema ... does not contain a key named
#   'antialiasing'
# which is a desktop theming mismatch, not a fault in this script. Tk never
# touches GSettings, so prefer it and fall back quietly.
for _backend in ('TkAgg', 'QtAgg', 'Agg'):
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        continue
import matplotlib.pyplot as plt      # noqa: E402

G = 9.81

SCHEMES = {
    '1': ('pure_pursuit', 'Pure Pursuit (PP)'),
    '2': ('deviated_pursuit', 'Deviated Pursuit (DP)'),
    '3': ('losg', 'Line of Sight Guidance (LOSG)'),
    '4': ('beam_rider', 'Beam Rider (BR)'),
    '5': ('pn', 'Proportional Navigation (PN)'),
    '6': ('tpn', 'True Proportional Navigation (TPN)'),
    '7': ('ppn', 'Pure Proportional Navigation (PPN)'),
}


# --------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------
def simulate(scheme, vp, vt, r0, theta0, alpha_t0, N, k, delta, at,
             r_cap=10.0, dt=0.01, t_max=120.0):
    """Fly one engagement. Returns time histories plus a summary."""
    xp, yp = 0.0, 0.0
    xt, yt = r0 * math.cos(theta0), r0 * math.sin(theta0)
    alpha_p, alpha_t = theta0, alpha_t0

    t, r = 0.0, r0
    hit = False
    r_min, t_rmin, pk_ap = r0, 0.0, 0.0
    d = {key: [] for key in (
        't', 'xp', 'yp', 'xt', 'yt', 'r', 'theta', 'theta_dot',
        'alpha_p', 'alpha_t', 'ap', 'at', 'vr', 'vth',
        'vxp', 'vyp', 'vxt', 'vyt', 'vrx', 'vry', 'nu', 'psi')}

    while t < t_max:
        dx, dy = xt - xp, yt - yp
        r = math.hypot(dx, dy)
        if r < r_min:
            r_min, t_rmin = r, t
        if r <= r_cap:
            hit = True
            break

        vxp, vyp = vp * math.cos(alpha_p), vp * math.sin(alpha_p)
        vxt, vyt = vt * math.cos(alpha_t), vt * math.sin(alpha_t)
        vrx, vry = vxt - vxp, vyt - vyp

        vr = (dx * vrx + dy * vry) / r          # range rate, dr/dt
        vth = (dx * vry - dy * vrx) / r         # transverse, r*theta_dot
        theta = math.atan2(dy, dx)
        theta_dot = vth / r

        # Station-to-target and station-to-pursuer LOS, for LOSG and BR.
        theta_t_stn = math.atan2(yt, xt)
        theta_p_stn = math.atan2(yp, xp) if (xp or yp) else theta_t_stn

        if scheme == 'pure_pursuit':
            alpha_p = theta
            ap = turn = vp * theta_dot
        elif scheme == 'deviated_pursuit':
            alpha_p = theta + delta
            ap = turn = vp * theta_dot
        elif scheme == 'losg':
            ap = turn = k * (theta_t_stn - theta_p_stn)
        elif scheme == 'beam_rider':
            ap = turn = k * vp * (theta_t_stn - theta_p_stn)
        elif scheme == 'pn':
            ap = turn = N * (-vr) * theta_dot
        elif scheme == 'tpn':                   # accel normal to the LOS
            ap = N * (-vr) * theta_dot
            turn = ap * math.cos(alpha_p - theta)
        elif scheme == 'ppn':                   # accel normal to velocity
            ap = turn = N * vp * theta_dot
        else:
            ap = turn = N * (-vr) * theta_dot

        pk_ap = max(pk_ap, abs(ap))

        for key, val in (('t', t), ('xp', xp), ('yp', yp), ('xt', xt),
                         ('yt', yt), ('r', r),
                         ('theta', math.degrees(theta)),
                         ('theta_dot', math.degrees(theta_dot)),
                         ('alpha_p', math.degrees(alpha_p)),
                         ('alpha_t', math.degrees(alpha_t)),
                         ('ap', ap / G), ('at', at / G),
                         ('vr', vr), ('vth', vth),
                         ('vxp', vxp), ('vyp', vyp),
                         ('vxt', vxt), ('vyt', vyt),
                         ('vrx', vrx), ('vry', vry),
                         ('nu', vp / vt),
                         ('psi', math.degrees(alpha_t - theta))):
            d[key].append(val)

        alpha_t += (at / vt) * dt
        if scheme not in ('pure_pursuit', 'deviated_pursuit'):
            alpha_p += (turn / vp) * dt
        xp += vxp * dt
        yp += vyp * dt
        xt += vxt * dt
        yt += vyt * dt
        t += dt

    d.update(hit=hit, t_f=t, r_f=r, r_miss=r_min, t_miss=t_rmin,
             peak_ap=pk_ap, nu_val=vp / vt, vp=vp, vt=vt)
    return d


# --------------------------------------------------------------------------
# Plottable variables
# --------------------------------------------------------------------------
VARS = [
    ('t', 'Time [s]'),
    ('xp', 'Pursuer X [m]'),
    ('yp', 'Pursuer Y [m]'),
    ('xt', 'Target X [m]'),
    ('yt', 'Target Y [m]'),
    ('r', 'Range r [m]'),
    ('theta', 'LOS angle theta [deg]'),
    ('theta_dot', 'LOS rate theta_dot [deg/s]'),
    ('ap', 'Pursuer latax ap [g]'),
    ('at', 'Target latax at [g]'),
    ('alpha_p', 'Pursuer heading alpha_p [deg]'),
    ('alpha_t', 'Target heading alpha_t [deg]'),
    ('vr', 'Radial rel vel vr [m/s]'),
    ('vth', 'Transverse rel vel vth [m/s]'),
    ('vxp', 'Pursuer vel X [m/s]'),
    ('vyp', 'Pursuer vel Y [m/s]'),
    ('vxt', 'Target vel X [m/s]'),
    ('vyt', 'Target vel Y [m/s]'),
    ('vrx', 'Rel vel X [m/s]'),
    ('vry', 'Rel vel Y [m/s]'),
    ('nu', 'Speed ratio nu = vp/vt'),
    ('psi', 'Psi = alpha_t - theta [deg]'),
]
KEYS = [k for k, _ in VARS]
LABELS = dict(VARS)


def print_vars():
    print('\n--- VARIABLES ---')
    for i in range(0, len(VARS), 2):
        left = f'{i + 1:2d}. {VARS[i][1]:<34}'
        if i + 1 < len(VARS):
            print(f'{left} | {i + 2:2d}. {VARS[i + 1][1]}')
        else:
            print(left)


def ask_var(prompt):
    """Accept either the menu number or the variable name."""
    while True:
        s = input(prompt).strip()
        if not s:
            print('    enter a number or a name')
            continue
        if s.isdigit() and 1 <= int(s) <= len(VARS):
            return KEYS[int(s) - 1]
        low = s.lower().replace(' ', '_')
        if low in KEYS:
            return low
        hits = [k for k in KEYS if k.startswith(low)]
        if len(hits) == 1:
            return hits[0]
        hits = [k for k, lab in VARS if low in lab.lower().replace(' ', '_')]
        if len(hits) == 1:
            return hits[0]
        print(f'    "{s}" is not a variable. Use a number 1-{len(VARS)}, or a '
              f'name like t, r, theta_dot, ap, nu')


def ask_float(prompt, default):
    s = input(f'  {prompt} [{default:g}]: ').strip()
    try:
        return float(s) if s else default
    except ValueError:
        print(f'    not a number, using {default:g}')
        return default


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def main():
    print('--- GUIDANCE SCHEMES ---')
    for num, (_, name) in SCHEMES.items():
        print(f'  {num}. {name}')
    sch = input('Select scheme (1-7): ').strip()
    scheme, scheme_name = SCHEMES.get(sch, SCHEMES['5'])

    print(f'\n--- SETTINGS for {scheme_name} ---')
    vp = ask_float('pursuer speed vp [m/s]', 400.0)
    r0 = ask_float('initial range r0 [m]', 7000.0)
    theta0 = math.radians(ask_float('initial LOS angle theta0 [deg]', 30.0))
    alpha_t0 = math.radians(ask_float('target heading alpha_t0 [deg]', 60.0))
    N = ask_float('navigation constant N', 5.0)
    k = ask_float('LOS / implementation gain k', 5.0)
    delta = math.radians(ask_float('deviation angle delta [deg]', 10.0))
    at = ask_float('target latax at [m/s^2]', 0.0)

    # Speed ratios. vp is held fixed and vt = vp / nu, so the pursuer's
    # capability stays constant and only the target changes -- which is how a
    # capturability study is normally set up.
    raw = input('\nSpeed ratios nu = vp/vt, comma separated [0.9,1.5,3.0]: ')
    raw = raw.strip() or '0.9,1.5,3.0'
    ratios = []
    for piece in raw.split(','):
        piece = piece.strip()
        if not piece:
            continue
        try:
            val = float(piece)
        except ValueError:
            print(f'  ignoring "{piece}" -- not a number')
            continue
        if val <= 0:
            print(f'  ignoring {val:g} -- nu must be positive')
            continue
        ratios.append(val)
    if not ratios:
        ratios = [1.5]

    # --- run one engagement per speed ratio ---------------------------
    runs = []
    print(f'\nSimulating {scheme_name} at nu = '
          + ', '.join(f'{n:g}' for n in ratios) + ' ...')
    for nu in ratios:
        runs.append(simulate(scheme, vp, vp / nu, r0, theta0, alpha_t0,
                             N, k, delta, at))

    # --- report -------------------------------------------------------
    print('\n' + '=' * 72)
    print(f'  {scheme_name}   vp = {vp:g} m/s   r0 = {r0:g} m   '
          f'theta0 = {math.degrees(theta0):g} deg')
    print('=' * 72)
    print(f'{"nu":>6} {"vt":>8} {"result":>8} {"R_miss":>10} '
          f'{"t_miss":>9} {"t_f":>8} {"peak|ap|":>10}')
    print('-' * 72)
    for run in runs:
        print(f'{run["nu_val"]:6.2f} {run["vt"]:8.1f} '
              f'{"HIT" if run["hit"] else "MISS":>8} '
              f'{run["r_miss"]:10.2f} {run["t_miss"]:9.2f} '
              f'{run["t_f"]:8.2f} {run["peak_ap"] / G:9.2f}g')
    print('=' * 72)
    print('  R_miss is the closest approach reached [m]; HIT means it came '
          'within the\n  10 m capture radius.')

    # --- plotting -----------------------------------------------------
    while True:
        print('\n--- PLOTTING ---')
        print('  1. one panel per speed ratio')
        print('  2. all speed ratios overlaid on one panel')
        print('  0. exit')
        mode = input('Choose [1]: ').strip() or '1'
        if mode == '0':
            break
        if mode not in ('1', '2'):
            continue

        print_vars()
        x_key = ask_var('\n  X-axis (number or name): ')
        y_key = ask_var('  Y-axis (number or name): ')

        if mode == '2':
            plt.figure(figsize=(8.0, 5.5))
            for run in runs:
                plt.plot(run[x_key], run[y_key], lw=1.8,
                         label=f'nu = {run["nu_val"]:g} '
                               f'({"hit" if run["hit"] else "miss"}, '
                               f'R_miss {run["r_miss"]:.1f} m)')
            plt.xlabel(LABELS[x_key])
            plt.ylabel(LABELS[y_key])
            plt.title(f'{scheme_name}: {LABELS[y_key]} vs {LABELS[x_key]}')
            plt.grid(True, ls=':', alpha=0.7)
            plt.legend(loc='best', fontsize=9)
            if {x_key, y_key} == {'xp', 'yp'}:
                plt.axis('equal')
        else:
            n = len(runs)
            rows, cols = (1, n) if n <= 3 else (2, (n + 1) // 2)
            fig, axes = plt.subplots(rows, cols,
                                     figsize=(5.0 * cols, 4.0 * rows),
                                     squeeze=False)
            flat = axes.ravel()
            for ax in flat[n:]:
                ax.axis('off')
            for ax, run in zip(flat, runs):
                ax.plot(run[x_key], run[y_key], lw=1.8, color='tab:blue',
                        label=LABELS[y_key])
                # Trajectory panels are much clearer with the target drawn in.
                if {x_key, y_key} == {'xp', 'yp'}:
                    ax.plot(run['xt'], run['yt'], lw=1.5, ls='--',
                            color='tab:red', label='Target')
                    ax.axis('equal')
                ax.set_xlabel(LABELS[x_key])
                ax.set_ylabel(LABELS[y_key])
                ax.set_title(f'nu = {run["nu_val"]:g}  '
                             f'({"HIT" if run["hit"] else "MISS"}, '
                             f'R_miss {run["r_miss"]:.1f} m)', fontsize=10)
                ax.grid(True, ls=':', alpha=0.7)
                ax.legend(loc='best', fontsize=8)
            fig.suptitle(f'{scheme_name}: {LABELS[y_key]} vs '
                         f'{LABELS[x_key]}', fontsize=12)

        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
