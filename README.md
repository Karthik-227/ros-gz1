# Guidance — Pure Pursuit & Proportional Navigation in Gazebo

A two-quadcopter **pursuit–evasion testbed** for AS5570 (Principles of Guidance for
Autonomous Vehicles, IIT Madras). Eight classical guidance laws, flown two ways: as a fast
point-mass simulation, and on real ArduPilot firmware in Gazebo Harmonic.

Both paths run the **same guidance code** and write the **same CSV format**, so a flown run
and a simulated one overlay on the same axes. That is the point of the project: it shows
what a guidance law does in theory *and* what an actual airframe with a 30° lean limit and
a 50 Hz control loop does with it.

```
Pursuer  ───── chases ─────▶  Target
  │                             │
  ArduPilot SITL #0        ArduPilot SITL #1
  └────────── Gazebo Harmonic ──┘
             ▲
             │ MAVLink @ 50 Hz
      guidance_sim  ── acceleration commands
```

---

## Guidance laws

| key | law | command |
|---|---|---|
| `pure_pursuit` | Pure pursuit | `a = k·Vp·(λ − γ)` |
| `deviated_pursuit` | Deviated (fixed-lead) pursuit | `a = k·Vp·(λ − γ − δ)` |
| `constant_bearing` | Constant bearing | lead recomputed each step from the collision triangle |
| `pn` / `tpn` | True proportional navigation | `a = N·Vc·λ̇`, ⊥ to the LOS |
| `ppn` | Pure proportional navigation | `a = N·Vp·λ̇`, ⊥ to the velocity |
| `apn` | Augmented PN | `a = N·Vc·λ̇ + (N/2)·a_T⊥` |
| `zem` | Zero-effort miss | `a = N·ZEM⊥ / t_go²` |
| `los` | CLOS / beam rider | PD on cross-beam error from a ground station |

All clip to `a_max = g·tan(30°) = 5.66 m/s²` — what an ArduCopter quad can actually deliver
at the stock `ANGLE_MAX`.

---

## Results

Crossing engagement, `r0 = 500 m`, `λ0 = 30°`, `γ_T = 200°`, `Vp = 12`, `Vt = 6`:

| law | outcome | miss (m) | t_f (s) | peak latax (m/s²) | Δv |
|---|---|---|---|---|---|
| `pure_pursuit` | miss | 2.80 | 28.22 | 5.66 (saturated) | 15.1 |
| `deviated_pursuit` | hit | 0.96 | 35.09 | 5.66 (saturated) | 47.8 |
| `constant_bearing` | hit | 0.83 | 28.00 | 5.66 | 7.5 |
| `pn` | hit | 0.91 | 28.76 | 0.63 | 11.8 |
| `ppn` | hit | 0.88 | 29.13 | 0.51 | 14.4 |
| `apn` | hit | 0.91 | 28.76 | 0.63 | 11.8 |
| `zem` | hit | 0.91 | 28.76 | 0.63 | 11.8 |
| `los` | hit | 0.96 | 28.02 | 5.65 | 10.7 |

Three findings worth the write-up:

- **Pure pursuit misses at every speed ratio, and the miss *grows* with it** — 1.06 m at
  ν = 0.3 up to 16.31 m at ν = 1.3 — while PN captures throughout at a peak latax of
  1.24 m/s². The classic tail-chase problem, measured.
- **PPN with N = 3 fails at ν = 1.3 where TPN does not.** PPN's gain uses `Vp` instead of
  the closing velocity `Vc`, so a fast evader drives it into saturation. N = 5 recovers it.
- **`pn`, `apn` and `zem` are numerically identical against a non-manoeuvring target** —
  and that is correct, not a bug: `a_T = 0` collapses APN to PN, and ZEM at N = 3 is
  algebraically PN at N = 3. It doubles as a free regression test.

An engagement has also been **flown end to end in Gazebo** with both copters airborne and
the pursuer intercepting under proportional navigation.

---

## Quick start

Already set up? Two terminals:

```bash
# Terminal A — simulator (leave running, wait for "ArduPilot Ready" twice)
cd ~/guidance_ws && source install/setup.bash && ros2 launch guidance_sim engagement.launch.py
```

```bash
# Terminal B — launch both drones, then fly the engagement
cd ~/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff_both --alt 50
cd ~/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pn --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_pn.csv
```

No simulator needed for the guidance study itself — this runs in about a second:

```bash
ros2 run guidance_sim offline --law pn --lam0 30 --gam-t 200 -o runs/pn.csv
```

**Starting from nothing? → [SETUP.md](SETUP.md)** walks a blank Ubuntu 22.04 machine all the
way to a flying engagement, assuming no prior ROS, Gazebo or ArduPilot experience.

---

## Documentation

| file | what it covers |
|---|---|
| **[SETUP.md](SETUP.md)** | Full install from a blank machine. Every command, with a "check it worked" after each step. Start here. |
| **[COMMANDS.md](COMMANDS.md)** | Command reference — every command in the order you use them, plus full flag tables. |
| **[GUIDE.md](GUIDE.md)** | Bring-up order, the conventions, and troubleshooting for every failure met so far. |
| **[WORKSPACE_HANDOFF.md](WORKSPACE_HANDOFF.md)** | Architecture, current state, and a phased plan for extending this into a delivery-drone simulation. |
| [src/guidance_sim/README.md](src/guidance_sim/README.md) | The theory: laws, frames, sign conventions, verified identities. |

---

## Requirements

- **Ubuntu 22.04 (Jammy)** — required, not optional
- **ROS 2 Humble**
- **Gazebo Harmonic** (gz-sim 8) — *not* Fortress
- **ArduPilot SITL** + **ardupilot_gazebo**, cloned separately into `src/` (see SETUP.md)
- Python: `numpy`, `scipy`, `matplotlib`, `pymavlink`, `MAVProxy`

ArduPilot and the Gazebo plugin are **not** vendored in this repository — they are large
upstream projects and SETUP.md clones them for you.

---

## Layout

```
guidance_ws/
├── src/guidance_sim/
│   ├── guidance_sim/
│   │   ├── engagement.py      geometry: R, Ṙ, Vc, λ, λ̇, t_go, ZEM
│   │   ├── guidance_laws.py   the 8 laws + registry
│   │   ├── offline.py         point-mass simulator and parameter sweep
│   │   ├── engage.py          live 50 Hz run against two SITL copters
│   │   ├── mavlink_io.py      pymavlink wrapper, acceleration setpoints
│   │   ├── takeoff.py         takeoff / land, single and both
│   │   ├── plot_run.py        comparison figures, capturability chart
│   │   └── runlog.py          the shared CSV schema
│   ├── config/    gains, engagement presets, SITL parameters
│   ├── worlds/    engagement.sdf — two copters, shared NED origin
│   ├── models/    iris_pursuer (FDM 9002), iris_target (FDM 9012)
│   ├── launch/    engagement.launch.py — Gazebo + both SITL instances
│   └── tools/     run_study.py — every report figure in one pass
└── computer_assignment/   standalone planar guidance scripts (separate coursework)
```

## Conventions

Inertial frame is **NED** (x North, y East, z Down) — what MAVLink reports, so no
conversion sits between telemetry and the maths. LOS angle `λ = atan2(dE, dN)`, bearings
from North. Latax is signed positive to the pursuer's right, so `γ̇ = latax / V`. Laws
return a **NED acceleration vector**, so the same code runs planar and 3-D.

## Licence

MIT.
