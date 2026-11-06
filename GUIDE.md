# guidance_ws — launch guide

Step-by-step bring-up for the AS5570 two-quadcopter guidance testbed.
Package source lives in `src/guidance_sim`; see `src/guidance_sim/README.md` for the
theory, the law list and the conventions.

## Which part do I use?

**Use A (offline) for anything about guidance.** Choosing a law, tuning `N` or `k`, asking
when a law captures, producing report figures. It runs in seconds, has no moving parts, and
is fully tested. Start here always — and for most questions, stop here.

**Use B (Gazebo + SITL) only to show a law flies on a real autopilot.** It adds ArduCopter's
controllers, attitude limits and timing on top of the same law. It is slow, has four moving
parts, and is only partly brought up. Go here once A already gives you the answer you want
to demonstrate.

Rule of thumb: if the question would have the same answer with a perfect vehicle, it belongs
in A. Only questions about the *vehicle* belong in B.

**Status (2026-11-06):** **both paths work.** A has always worked. B is now flown end to
end — the world loads, both SITL instances arm, `takeoff_both` launches both copters, and
a PN engagement has run to a hit. Part C is still the right order to bring it up in after
a restart, and Part D lists every failure met on the way there.

**For the commands alone, with no explanation, see [COMMANDS.md](COMMANDS.md).**

Three fixes were needed to get B flying, all documented in Part D and worth knowing before
editing anything:

1. `config/sitl_guided.parm` now carries `FRAME_CLASS`/`FRAME_TYPE` — ArduPilot 4.8-dev no
   longer supplies per-frame defaults for `--model JSON`, so without them nothing arms.
2. `engage --vp` flies the pursuer up to cruise speed before guidance starts. **No
   classical guidance law can create speed**; they only steer an existing velocity vector,
   and a SITL copter starts hovering at zero.
3. The target's altitude is now trimmed to `--alt`. It used to hold whatever height it
   happened to have, and a vertical gap floors the miss distance at that gap.

---

## 0. Before every session

Setup is already done. Each new shell just needs the overlay:

```bash
cd /home/karthik/guidance_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```

After editing package code, rebuild:

```bash
cd /home/karthik/guidance_ws && colcon build --packages-select guidance_sim --symlink-install
```

---

## A. Offline simulation (no Gazebo, no SITL)

This is the fast loop. Every guidance question that is actually about guidance should be
answered here first — debugging a control law through a flight simulator is a waste of an
afternoon.

### A1. One engagement

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim offline --law pn --lam0 30 --gam-t 200 -o runs/pn.csv
```

Prints a one-line summary and writes the CSV:

```
pn               hit      miss=   0.91 m  t_f= 28.76 s  peak|latax|= 0.63 m/s^2  mean= 0.40  dv=  11.8 m/s
```

Useful flags (full list with `--help`):

| flag | meaning |
|---|---|
| `--law` | `pure_pursuit`, `deviated_pursuit`, `constant_bearing`, `pn`/`tpn`, `ppn`, `apn`, `zem`, `los` |
| `-N` | navigation ratio for the PN family (default 3) |
| `-k` | heading gain for the pursuit family (default 2) |
| `--a-max` | latax limit, default 5.66 = g·tan(30°) |
| `--r0 --lam0 --gam0 --gam-t` | geometry: range m, LOS bearing deg, headings deg from North |
| `--vp --vt` | speeds, m/s |
| `--target` | `straight`, `turn`, `weave` |
| `--at --weave-period` | target manoeuvre amplitude m/s² and period s |
| `--lead-start` | start the pursuer on the collision triangle |
| `--dt --t-max --capture` | solver step, time limit, capture radius |

### A2. Compare several laws on one figure

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && for law in pure_pursuit pn ppn los; do ros2 run guidance_sim offline --law $law --lam0 30 --gam-t 200 -o runs/$law.csv; done
```

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/pn.csv runs/pure_pursuit.csv runs/ppn.csv runs/los.csv -o runs/compare.png
```

Five panels: plan view (NED), range, LOS rate, signed latax against its saturation bound,
and heading error. Add `--show` for an interactive window instead of only a PNG.

### A3. Capturability sweep

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim sweep --laws pure_pursuit,constant_bearing,pn,ppn --n-list 3,5 --nu-list 0.3,0.5,0.7,0.9,1.1,1.3 -o runs/sweep.csv
```

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/sweep.csv -o runs/sweep.png
```

`plot_run` auto-detects a sweep summary and draws miss distance and peak latax against
speed ratio instead. Filled marker = captured, hollow = missed.

### A4. Regenerate every report figure in one pass

```bash
cd /home/karthik/guidance_ws/src/guidance_sim && python3 tools/run_study.py --out-dir study
```

Runs all 8 laws over 5 engagement presets plus the sweep: 76 runs, 6 PNGs, about a minute.

---

## B. Gazebo + dual SITL

**Partly tested.** The world and the two `sim_vehicle.py` commands below are known
good. The engagement loop on top of them is not. Work through Part C the first time
rather than running this blind.

Needs four terminals. Each one needs the overlay sourced (step 0).

### B1. Everything at once (ROS 2 launch)

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 launch guidance_sim engagement.launch.py
```

Starts Gazebo on `worlds/engagement.sdf`, waits 6 s, starts SITL instance 0, waits another
6 s, starts instance 1. Add `headless:=true` to skip the GUI.

Then, in a second terminal:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pn --takeoff --alt 50 -o runs/sitl_pn.csv
```

### B1a. Take off both copters on one command

`engage --takeoff` launches both vehicles as part of the run. To put them in the air on
their own, without flying an engagement — which is what you want for the first-run
checklist, for a demo, or any time you just want to see both copters hovering:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff_both --alt 50
```

Both are set to GUIDED and **armed first**, then both takeoffs are commanded, and only
then does it wait for the climbs — so they lift off together rather than one after the
other. A pre-arm failure is therefore reported with nothing yet in the air.

| flag | meaning |
|---|---|
| `--alt` | target altitude above home, m (default 50) |
| `--prompt` | arm both, then wait for Enter before launching — use it to start a screen recording on the mark |
| `--pursuer-url` / `--target-url` | MAVLink links (defaults `udp:127.0.0.1:14550` / `:14560`) |
| `--timeout` | per-vehicle climb timeout, s (default 90) |

Expected output:

```
[pursuer] mode GUIDED
[target] mode GUIDED
[pursuer] armed
[target] armed
[pursuer] takeoff commanded to 50 m
[target] takeoff commanded to 50 m
[pursuer] at 50.0 m
[target] at 50.0 m
[run] both airborne: pursuer 50.0 m, target 50.0 m
```

Bring them both down again with:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim land_both
```

`land_both` takes `--rtl` to return to launch instead of landing in place, and lands the
second copter even if the first times out — leaving one hovering because its partner
misbehaved is worse than reporting two failures. Single-vehicle `takeoff` and `land`
(with `--url`) are unchanged.

### B2. Manual, four terminals

Prefer this while debugging — you see each component's output separately.

**Terminal 1 — Gazebo:**

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && GZ_SIM_RESOURCE_PATH=$PWD/install/guidance_sim/share/guidance_sim/models:$GZ_SIM_RESOURCE_PATH gz sim -v4 -r install/guidance_sim/share/guidance_sim/worlds/engagement.sdf
```

**Terminal 2 — pursuer, SITL instance 0:**

```bash
cd /home/karthik/guidance_ws/src/ardupilot && ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I 0 --custom-location=-35.363262,149.165237,584,353 --out=udp:127.0.0.1:14550 --no-rebuild --add-param-file=/home/karthik/guidance_ws/install/guidance_sim/share/guidance_sim/config/sitl_guided.parm --use-dir=/home/karthik/guidance_ws/sitl_state/pursuer
```

**Terminal 3 — target, SITL instance 1:**

```bash
cd /home/karthik/guidance_ws/src/ardupilot && ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I 1 --custom-location=-35.363262,149.165237,584,353 --out=udp:127.0.0.1:14560 --no-rebuild --add-param-file=/home/karthik/guidance_ws/install/guidance_sim/share/guidance_sim/config/sitl_guided.parm --use-dir=/home/karthik/guidance_ws/sitl_state/target
```

**Terminal 4 — the engagement:**

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pn --takeoff --alt 50 --vt 6 --gam-t 180 -o runs/sitl_pn.csv
```

### The one thing you must not get wrong

**Both SITL instances need the SAME `--custom-location`.** The Gazebo plugin reports `WorldPose`,
which is world-origin relative, so a shared home puts both copters in one common NED frame
and their `LOCAL_POSITION_NED` streams can be differenced directly. Different homes still
fly fine and produce an engagement that looks entirely plausible and is quietly wrong.

### Where the target actually starts

**Settled by measurement, 2026-11-05: the target starts 500 m due EAST.** A flown run
logged `lam = 1.586 rad = 90.9°` at t = 0, with `p_t_e = 500.0` and `p_t_n = -7.8`. The
comment inside `worlds/engagement.sdf` claiming "500 m North" is **wrong** — ignore it.

Gazebo's world here is ENU and the plugin maps it to NED as `(N, E, D) = (y, x, -z)`. The
target include is `<pose>500 0 ...</pose>` — Gazebo *x* — so it starts **500 m due East**
of the pursuer, not North.

`offline` measures `--lam0` as a bearing from North, so its default puts the target 500 m
**North**. The two start geometries are rotated 90° apart. For the PN family this is
harmless — range, miss distance and latax still overlay — but N/E tracks will not.

To make them match, set the include in `worlds/engagement.sdf` to
`<pose degrees="true">0 500 0.195 0 0 180</pose>` and rebuild.

### Port map

| role | instance | FDM port | MAVLink out | model |
|---|---|---|---|---|
| pursuer | `-I 0` | 9002 | `udp:127.0.0.1:14550` | `iris_pursuer` |
| target | `-I 1` | 9012 | `udp:127.0.0.1:14560` | `iris_target` |

The stride is 10, not 1 — `sim_vehicle.py` assigns `from_ardupilot_port = 9002 + 10·i`.

### Comparing a flown run against the simulation

Both paths write the same CSV schema, so they overlay directly:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/sitl_pn.csv runs/pn.csv -o runs/sitl_vs_offline.png
```

---

## C. First-run checklist

Do these in order. Each isolates one failure mode; skipping ahead just makes the failure
harder to attribute.

**1. World loads, both copters spawned.** Run Terminal 1, then check from another shell:

```bash
gz model --list
```

Expect `ground_plane`, `axes`, `iris_pursuer`, `iris_target`. **`iris_target` will not be
visible in the viewport** — it is 500 m out, so it is about one pixel on the horizon. That
is normal. Right-click it in the Entity Tree and choose **Move to** to see it. Only if it is
missing from `gz model --list` is anything wrong, and then `GZ_SIM_RESOURCE_PATH` is not
pointing at the installed `models/`.

**2. One SITL instance connects.** Run Terminal 2. If it sits waiting for the FDM, the
`fdm_port_in` in `models/iris_pursuer/model.sdf` does not match what instance 0 expects.

**3. Second instance connects.** Run Terminal 3. Both copters should now be live.

**4. Telemetry sanity, before any guidance.** Confirm both `LOCAL_POSITION_NED` streams
arrive and that differencing them gives `north ≈ 0, east ≈ 500` — East, not North; see
"Where the target actually starts" above. If this is wrong, nothing downstream can be right.

**5. Takeoff alone.** One vehicle first — if arming is going to fail, fail it on one
link with a short command, not inside a two-vehicle run:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff --url udp:127.0.0.1:14550 --alt 50
```

**5b. Both together.** Once one works, this is the real test that the pair is healthy:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff_both --alt 50
```

Both should report `armed`, then both `at 50.0 m`. If one arms and the other does not,
the problem is per-instance state (`sitl_state/<role>/eeprom.bin`) rather than the
shared param file — see Part D. Land them with `land_both` before moving on.

**6. Altitude hold alone.** Run `engage` with `--law pn --t-max 10` and watch whether the
pursuer holds 50 m. If it sinks or climbs away, suspect the signs or gains in
`engage.altitude_hold()` before suspecting the guidance law.

**7. Full engagement.** Then overlay against the offline run of the same geometry and
compare miss distance and peak latax.

---

## D. Troubleshooting

**`sim_vehicle.py` dies with `no such option: --home`.** `--home` belongs to the
`arducopter` binary, not to the `sim_vehicle.py` wrapper. The wrapper spells it
`--custom-location` and passes it on as `--home` itself. Its suggestion of
`--sim_vehicle_sh_compatible` is a red herring. The process dies instantly, so the first
instance's failure usually scrolls away and `engage` just sits on `waiting for heartbeat`.

**`error: failed to arm (pre-arm checks?)`, and the SITL terminal shows `AP: Frame:
UNSUPPORTED` at startup followed by `PreArm: Motors: Check frame class and type`.**
**Fixed — but read this, because it will come back if `sitl_guided.parm` is edited
carelessly.**

ArduPilot 4.8-dev changed how per-frame defaults reach the SITL binary. From
`Tools/autotest/sim_vehicle.py`, just above where it builds `--defaults`:

> *the SITL binary resolves the per-frame default parameter files from its embedded
> vehicleinfo.json keyed by `--model`, so we no longer pass `--defaults` here. Use
> `--add-param-file` or `--param` to layer extra parameters on top of the embedded
> defaults.*

Keyed by **`--model`, not by `-f`/`--frame`.** We pass `--model JSON` because that is
the FDM protocol the ArduPilot Gazebo plugin speaks, so the binary looks up frame
`"JSON"` and never loads `default_params/copter.parm` or
`default_params/gazebo-iris.parm` — and `gazebo-iris.parm` is the file that contains
`FRAME_CLASS 1` / `FRAME_TYPE 1`. `FRAME_CLASS` stays at 0, the frame is unsupported,
and **the motor/frame pre-arm check is not skippable by `ARMING_CHECK 0`**.

Confirm it from the SITL process's own command line, which it prints on startup:

```
arducopter --model JSON ... --defaults /…/share/guidance_sim/config/sitl_guided.parm -I0 …
```

If `--defaults` lists **only** `sitl_guided.parm`, the frame defaults are not being
loaded — that is this failure.

The fix is that `config/sitl_guided.parm` now carries everything from those two files
that matters: `FRAME_CLASS`/`FRAME_TYPE`, `MOT_THST_HOVER`/`MOT_THST_EXPO`, the INS
calibration offsets, and the fence settings. Three consequences:

1. **`sitl_guided.parm` is now load-bearing.** Do not remove the frame block.
2. **Rebuild after editing it** — it installs via `data_files`, and SITL reads the
   *installed* copy.
3. **A stale `eeprom.bin` beats the defaults file.** Saved parameters take priority over
   defaults, so if a run has already stored `FRAME_CLASS 0`, wipe the stored state once:

```bash
rm -f /home/karthik/guidance_ws/sitl_state/pursuer/eeprom.bin /home/karthik/guidance_ws/sitl_state/target/eeprom.bin
```

A secondary cause with the identical symptom, now prevented: the two instances sharing
one `eeprom.bin`. `sim_vehicle.py` only makes a per-instance storage directory when a
*single* process drives several instances (`-I 0,1` in one call); launched as two
separate processes — which is what both the manual terminals below and
`engagement.launch.py` do — each process sees itself as a lone instance and writes
`eeprom.bin` into its current directory, so the two corrupt each other. Fixed by giving
each instance its own `--use-dir`. A pile of `eeprom.bin.bak-*` files side by side is
the tell. Old shared state, if any is left:

```bash
rm -f /home/karthik/guidance_ws/eeprom.bin /home/karthik/guidance_ws/eeprom.bin.bak-* \
      /home/karthik/guidance_ws/src/ardupilot/eeprom.bin /home/karthik/guidance_ws/src/ardupilot/eeprom.bin.bak-*
```

**Pursuer spins on the spot / circles slowly and the range grows.** The pursuer has no
cruise speed. **Every classical guidance law only steers an existing velocity vector** —
PN is `a = N·Vc·λ̇`, pure pursuit is `a = k·Vp·ε`; none of them can produce speed. The
offline simulator never shows this because `InitialConditions.states()` starts the
pursuer at `Vp` by construction, whereas a SITL copter after takeoff is hovering at zero.

With `|v_p| ≈ 0`, `right_normal(v_p)` normalises a near-zero vector so the command
direction comes from telemetry noise, and `γ̇ = latax/|v|` explodes — the heading spins
while the copter barely moves. The tell in the CSV is `Vp` near zero, `Rdot` positive and
`t_go` showing `inf` for the whole run.

Fixed by `--vp`: `engage` now flies the pursuer up to cruise speed on velocity setpoints
*before* guidance starts, and holds that speed with an along-track term during the run
(the throttle half of the job — the law steers, `speed_hold()` maintains the speed the law
assumes it has). Guidance keeps priority inside the `g·tan(ANGLE_MAX)` horizontal
envelope; speed-hold takes the remainder.

| flag | meaning |
|---|---|
| `--vp` | pursuer cruise speed, m/s (default 12, matching the offline default) |
| `--gam0` | initial pursuer heading, deg from North (default: straight at the target) |
| `--boost` | seconds allowed to reach cruise speed before guidance starts (default 15) |
| `--speed-gain` | along-track gain holding `\|v_p\|` at `--vp` (default 0.5) |
| `--a-speed` | cap on the speed-hold command, m/s² (default 2) |

It prints `[run] pursuer at 11.84 m/s` before engaging, and warns if it reached less than
half of what was asked — in which case check `WPNAV_SPEED` and `ANGLE_MAX`, or raise
`--boost`. **Keep `--vp` equal to the `--vp` used in the offline run you are comparing
against**, or the comparison is meaningless.

**Copter drifts to a hover mid-engagement.** The acceleration setpoint stream stalled.
ArduCopter zeroes the velocity and acceleration targets after `GUID_TIMEOUT` (3 s) and
brakes. Check the achieved loop rate that `engage` prints on exit — if it is well under
50 Hz, something is blocking the loop.

**Copter ignores the command and holds position.** The type mask is wrong. Only
`pos_ignore && vel_ignore && !acc_ignore` reaches `set_accel_NED_mss()`; everything else
falls through to `hold_position()`. The constant is `ACCEL_TYPEMASK = 3135` in
`mavlink_io.py` and should not be edited.

**Pursuer loses altitude during hard turns.** Expected and handled — `engage.budget()`
reserves the vertical share of `a_max` and gives the remainder to the horizontal plane, so
a saturated turn cannot starve altitude hold. If it still sinks, raise `ANGLE_MAX` (see
below) rather than widening the altitude clip.

**Latax saturates earlier in SITL than offline.** `ANGLE_MAX` defaults to 30°, so the
airframe can only deliver g·tan(30°) = 5.66 m/s². That is the package default. Raising
`ANGLE_MAX` to 45 in `config/sitl_guided.parm` buys 9.81 m/s² — but then offline runs need
`--a-max 9.81` too, or the comparison is meaningless.

**Arming fails.** `sitl_guided.parm` sets `ARMING_CHECK 0`, which is a simulation
convenience only. If you did not load the param file, pre-arm checks will block arming
until the EKF settles.

**Tracking looks over-damped.** Try `GUID_OPTIONS 32` (`DoNotStabilizeVelocityXY`) for a
more open-loop acceleration response. Default 0 keeps position and velocity error feedback
active around the integrated target and is the right starting point.

**`ros2 run` cannot find the executable.** The overlay is not sourced, or the build is
stale. Re-run step 0.
