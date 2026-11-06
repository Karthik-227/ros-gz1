# guidance_sim

Two-quadcopter guidance testbed for **AS5570 — Principles of Guidance for Autonomous
Vehicles** (IIT Madras). A library of classical guidance laws, an offline point-mass
simulator for fast parameter studies, and a live runner that flies the same laws on two
ArduPilot SITL copters in Gazebo Harmonic.

Both paths emit the **same CSV schema**, so a flown run and a simulated one overlay
directly in the same plot.

## Layout

```
guidance_sim/
├── engagement.py      Engagement geometry: R, Rdot, Vc, lambda, lambdadot, t_go, ZEM
├── guidance_laws.py   8 laws + registry (make_law)
├── offline.py         Point-mass simulator and parameter sweep
├── plot_run.py        Five-panel comparison figures and the capturability chart
├── runlog.py          Shared CSV schema (read and write)
├── mavlink_io.py      pymavlink wrapper: accel setpoints, arm, takeoff, telemetry
├── takeoff.py         takeoff / land entry points
└── engage.py          Live two-vehicle run against SITL
config/   laws.yaml (gain and engagement presets), sitl_guided.parm
worlds/   engagement.sdf — two copters, open plane, shared NED origin
models/   iris_pursuer (FDM 9002), iris_target (FDM 9012)
launch/   engagement.launch.py — Gazebo + both SITL instances
tools/    run_study.py — regenerate every report figure in one pass
```

## Conventions

Inertial frame is **NED** (x North, y East, z Down), which is what MAVLink's
`LOCAL_POSITION_NED` reports, so no conversion sits between the telemetry and the maths.
An altitude of 50 m is `z = -50`.

| symbol | meaning | convention |
|---|---|---|
| `lam` (λ) | LOS angle | `atan2(dE, dN)`, from North, positive toward East |
| `gam` (γ) | pursuer heading | same bearing convention |
| `eps` (ε) | heading error | `lam - gam` |
| `lamdot` (λ̇) | LOS rate | Down component of `omega = (r × v_rel) / R²` |
| latax | lateral acceleration | signed scalar, **positive to the pursuer's right**, so `gamdot = latax / V` |

A law returns a **NED acceleration vector**; `eng.latax_of(a_cmd)` recovers the signed
scalar for plotting. Laws are written in vector form, so the same code runs planar and 3D.

## Guidance laws

| key | law | command |
|---|---|---|
| `pure_pursuit` | Pure pursuit | `a = K·Vp·(λ − γ)` |
| `deviated_pursuit` | Deviated (fixed-lead) pursuit | `a = K·Vp·(λ − γ − δ)` |
| `constant_bearing` | Constant bearing | lead recomputed each step from the collision triangle |
| `pn` / `tpn` | True PN | `a = N·Vc·λ̇`, ⊥ to the LOS |
| `ppn` | Pure PN | `a = N·Vp·λ̇`, ⊥ to the velocity |
| `apn` | Augmented PN | `a = N·Vc·λ̇ + (N/2)·a_T⊥` |
| `zem` | Zero-effort miss | `a = N·ZEM⊥ / t_go²` |
| `los` | CLOS / beam rider | PD on cross-beam error from a ground station |

All clip to `a_max`, which defaults to **5.66 m/s² = g·tan(30°)** — what an ArduCopter quad
can actually deliver at the stock `ANGLE_MAX` of 30°. Raising `ANGLE_MAX` to 45° in
`config/sitl_guided.parm` buys 9.81 m/s². Offline and SITL runs must use the same bound or
the comparison is meaningless.

## Quick start — offline

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim offline --law pn --lam0 30 --gam-t 200 -o runs/pn.csv
```

Compare several laws on one figure:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/pn.csv runs/pure_pursuit.csv -o runs/compare.png
```

Capturability sweep (law × N × speed ratio):

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim sweep --laws pure_pursuit,pn,ppn --n-list 3,5 --nu-list 0.3,0.7,1.1,1.3 -o runs/sweep.csv
```

Regenerate every report figure at once:

```bash
cd /home/karthik/guidance_ws/src/guidance_sim && python3 tools/run_study.py --out-dir study
```

## Quick start — Gazebo + SITL

Launch Gazebo and both SITL instances:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 launch guidance_sim engagement.launch.py
```

Then fly an engagement:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pn --takeoff --alt 50 -o runs/sitl_pn.csv
```

**Both SITL instances must use the same `--custom-location`.** The Gazebo plugin reports `WorldPose`
(world-origin relative), so a shared home puts both vehicles in one NED frame and their
`LOCAL_POSITION_NED` streams can be differenced directly. The launch file does this; if you
start SITL by hand, don't forget it.

Ports follow `sim_vehicle.py`'s `from_ardupilot_port = 9002 + 10·i`:

| role | instance | FDM port | MAVLink |
|---|---|---|---|
| pursuer | `-I 0` | 9002 | `udp:127.0.0.1:14550` |
| target | `-I 1` | 9012 | `udp:127.0.0.1:14560` |

## How the acceleration command reaches the copter

`SET_POSITION_TARGET_LOCAL_NED` with position, velocity, yaw and yaw-rate masked off
(`type_mask = 3135 = 0xC3F`, `MAV_FRAME_LOCAL_NED`) routes to
`ModeGuided::set_accel_NED_mss()`, which feeds `AC_PosControl::input_accel_NE_m()` — a true
acceleration input, with velocity and position free-running as the controller integrates
it. Any mask combination ArduCopter does not recognise silently falls through to
`hold_position()`.

The setpoint **must be resent continuously**: ArduCopter zeroes both the velocity and
acceleration targets and brakes to a hover if nothing arrives within `GUID_TIMEOUT` (3 s).
`engage.py` runs at 50 Hz.

The Down axis is accel-controlled too, so commanding `afz = 0` requests zero *vertical
acceleration*, not altitude hold. By default `engage.py` discards the law's Down component
and drives that channel with a PD altitude hold; `--vertical law` hands it back for 3D work.

## Verified results

Crossing engagement (`--r0 500 --lam0 30 --gam-t 200 --vp 12 --vt 6`, `a_max` 5.66):

| law | outcome | miss (m) | t_f (s) | peak latax | Δv |
|---|---|---|---|---|---|
| `pure_pursuit` | miss | 2.80 | 28.22 | 5.66 (saturated) | 15.1 |
| `deviated_pursuit` | hit | 0.96 | 35.09 | 5.66 (saturated) | 47.8 |
| `constant_bearing` | hit | 0.83 | 28.00 | 5.66 | 7.5 |
| `pn` | hit | 0.91 | 28.76 | 0.63 | 11.8 |
| `ppn` | hit | 0.88 | 29.13 | 0.51 | 14.4 |
| `apn` | hit | 0.91 | 28.76 | 0.63 | 11.8 |
| `zem` | hit | 0.91 | 28.76 | 0.63 | 11.8 |
| `los` | hit | 0.96 | 28.02 | 5.65 | 10.7 |

`pn`, `apn` and `zem` are identical **on purpose** here: against a non-manoeuvring target
`a_T = 0` collapses APN to PN, and ZEM with `N = 3` is algebraically PN with `N = 3`. If
they ever differ on a `--target straight` run, something is broken.

Analytic identities checked numerically:

- `PN(N=3) ≡ ZEM(N=3)` to 0.0 difference
- `PN = N·Vc·λ̇·right_normal(r)` and `PPN = N·Vp·λ̇·right_normal(v_p)`
- `|ZEM⊥| = R²·λ̇ / Vc`
- `collision_heading()` for Vt=5, Vp=10 crossing gives `asin(0.5) = 30°`

Two results worth putting in the report:

**Capturability.** Pure pursuit misses at every speed ratio and the miss *grows* with it
(1.06 → 16.31 m as ν goes 0.3 → 1.3), while PN captures throughout at a peak latax of
1.24 m/s². **PPN with N=3 fails at ν = 1.3** (miss 1.40 m, saturated) where TPN does not —
PPN's gain uses `Vp` instead of `Vc`, so a fast evader drives it into saturation. PPN with
N=5 recovers (0.87 m, peak 1.05).

**APN against a manoeuvring target.** On the weaving engagement APN cuts peak latax from
5.52 to 4.71 m/s² versus PN, at the cost of much higher total effort (Δv 77.3 vs 21.0) —
it anticipates the weave continuously instead of reacting to it at the endgame.

## Dependencies

numpy, matplotlib, pymavlink. **Not pandas** — the code uses stdlib `csv`.
