# guidance_ws — command reference

Every command you need, in the order you actually use them. Copy-paste ready.
For *why* any of it works this way, see [GUIDE.md](GUIDE.md) (bring-up, troubleshooting)
and [WORKSPACE_HANDOFF.md](WORKSPACE_HANDOFF.md) (architecture).

**Status:** offline simulation works. Gazebo + dual SITL works — an engagement has been
flown end to end with both copters hitting.

---

## 0. The short version

Four terminals' worth of work in four commands. Terminal A stays running; B does the rest.

```bash
# A — bring up Gazebo + both SITL instances (leave running)
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 launch guidance_sim engagement.launch.py
```

```bash
# B — wait for both to print "ArduPilot Ready", then launch both copters
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff_both --alt 50
```

```bash
# B — fly the engagement
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pn --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_pn.csv
```

```bash
# B — bring them home
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim land_both
```

---

## 1. Every new shell

```bash
cd /home/karthik/guidance_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```

Every command below assumes this has been done. `ros2 run: executable not found` almost
always means it has not.

---

## 2. Build

```bash
cd /home/karthik/guidance_ws && colcon build --packages-select guidance_sim --symlink-install
```

**Rebuild after editing anything** — Python, SDF worlds, models, `.parm` files or launch
files. The installed tree holds copies, and SITL and Gazebo read the *installed* copy, not
your source.

If a change seems to have no effect, the install is stale. Clear just this one package:

```bash
cd /home/karthik/guidance_ws && rm -rf build/guidance_sim install/guidance_sim && colcon build --packages-select guidance_sim --symlink-install
```

Confirm a rebuild took by checking for a flag you know is new:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --help | grep -c vp
```

---

## 3. Bring up the simulator

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 launch guidance_sim engagement.launch.py
```

Gazebo, then SITL instance 0 at +6 s, then instance 1 at +12 s. **Wait for both to print
`AP: ArduPilot Ready`** before anything else — commands sent earlier just time out waiting
for a heartbeat.

Headless (no GUI, much faster):

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 launch guidance_sim engagement.launch.py headless:=true
```

Check both models actually spawned:

```bash
gz model --list
```

Expect `ground_plane`, `axes`, `iris_pursuer`, `iris_target`. **`iris_target` is invisible
in the viewport** — it is 500 m away, about one pixel. Right-click it in the Entity Tree →
**Move to**. Only its absence from `gz model --list` is a problem.

---

## 4. Fly

### Both copters up, on one command

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff_both --alt 50
```

Both are armed first, then both takeoffs are commanded, then it waits — so they lift off
together, and a pre-arm failure is reported with nothing in the air.

| flag | meaning |
|---|---|
| `--alt` | altitude above home, m (default 50) |
| `--prompt` | arm both, then wait for Enter before launching |
| `--timeout` | per-vehicle climb timeout, s (default 90) |
| `--pursuer-url` / `--target-url` | links (default `udp:127.0.0.1:14550` / `:14560`) |

### The engagement

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pn --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_pn.csv
```

Omit `--takeoff` if you already used `takeoff_both`; add it to have `engage` do the
arm-and-climb itself. Never both.

It prints a status line every second:

```
     t      R     Rdot      Vp     Vt   lamdot    latax   alt_p  alt_t
   0.0   500.8   -5.98   12.01   6.00   0.0121    0.223    50.0   50.0
```

**`Rdot` negative means closing.** If it stays positive, the engagement is opening and
something is wrong — see GUIDE.md Part D.

### Land

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim land_both
```

Add `--rtl` to return to launch instead of landing in place.

### Single vehicle, when you need it

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff --url udp:127.0.0.1:14550 --alt 50
```

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim land --url udp:127.0.0.1:14550
```

---

## 5. Results

Plot one flown run (five panels: plan view, range, LOS rate, signed latax vs its
saturation bound, heading error):

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/sitl_pn.csv --show
```

Overlay a flown run against the offline run of the same geometry — the comparison that
makes the whole testbed worth having:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim offline --law pn --r0 500 --lam0 90 --gam-t 180 --vp 12 --vt 6 -o runs/pn_offline.csv && ros2 run guidance_sim plot_run runs/sitl_pn.csv runs/pn_offline.csv -o runs/sitl_vs_offline.png --show
```

`--lam0 90` because **the target starts due East**, not North. Match `--vp`/`--vt`/`--r0`
to the flown run or the comparison means nothing.

Read a run's outcome straight out of the CSV header:

```bash
head -20 /home/karthik/guidance_ws/runs/sitl_pn.csv
```

The `#` comment lines carry law, gains, geometry, `outcome`, `miss`, `t_f`, `peak_latax`
and `total_dv`.

---

## 6. Offline — no Gazebo, no SITL

Seconds per run instead of minutes. **Answer every guidance question here first.**

One engagement:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim offline --law pn --lam0 30 --gam-t 200 -o runs/pn.csv
```

All eight laws on one figure:

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && for law in pure_pursuit deviated_pursuit constant_bearing pn ppn apn zem los; do ros2 run guidance_sim offline --law $law --lam0 30 --gam-t 200 -o runs/$law.csv; done
```

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/pn.csv runs/ppn.csv runs/pure_pursuit.csv runs/los.csv -o runs/compare.png --show
```

Capturability sweep (law × N × speed ratio; filled marker = captured, hollow = missed):

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim sweep --laws pure_pursuit,constant_bearing,pn,ppn --n-list 3,5 --nu-list 0.3,0.5,0.7,0.9,1.1,1.3 -o runs/sweep.csv && ros2 run guidance_sim plot_run runs/sweep.csv -o runs/sweep.png --show
```

Every report figure in one pass (76 runs, 6 PNGs, about a minute):

```bash
cd /home/karthik/guidance_ws/src/guidance_sim && python3 tools/run_study.py --out-dir study
```

---

## 7. Diagnostics

Are the SITL instances alive?

```bash
pgrep -af 'arducopter|sim_vehicle.py' | head
```

Nothing listed = the launch died; restart it. One listed = one instance crashed; check the
launch terminal.

Who holds the MAVLink ports? (`udp:` *binds*, so a leftover process blocks the next one
and you get an endless `waiting for heartbeat`.)

```bash
ss -lunp | grep -E '1455|1456'
```

Watch live velocities, attitude and a map alongside a run — needs a second `--out` added
to `_sitl()` in `launch/engagement.launch.py` first:

```bash
mavproxy.py --console --map --master=udp:127.0.0.1:14551
```

Gazebo ground truth, bypassing the EKF entirely:

```bash
gz topic -e -t /world/engagement/dynamic_pose/info -n 1
```

```bash
gz model -m iris_pursuer -p
```

Wipe stored SITL parameters so `sitl_guided.parm` is applied cleanly (needed after
changing frame parameters):

```bash
cd /home/karthik/guidance_ws && rm -f sitl_state/pursuer/eeprom.bin sitl_state/target/eeprom.bin
```

---

## 8. Flag reference

### `engage` — live engagement

| group | flag | default | meaning |
|---|---|---|---|
| law | `--law` | `pn` | `pure_pursuit`, `deviated_pursuit`, `constant_bearing`, `pn`/`tpn`, `ppn`, `apn`, `zem`, `los` |
| | `-N` | 3 | navigation ratio, PN family |
| | `-k` | 2 | heading gain, pursuit family |
| | `--delta` | 20 | deviated-pursuit lead angle, deg |
| | `--kp` / `--kd` | 1 / 2.5 | CLOS gains |
| | `--a-max` | 5.66 | latax limit, m/s² = g·tan(30°) |
| pursuer | `--vp` | 12 | **cruise speed. Guidance cannot start from a hover** |
| | `--gam0` | at the target | initial heading, deg from North |
| | `--boost` | 15 | seconds allowed to reach cruise speed |
| | `--speed-gain` | 0.5 | along-track gain holding `\|v_p\|` |
| | `--a-speed` | 2 | cap on the speed-hold command, m/s² |
| run | `--alt` | 50 | engagement altitude, m |
| | `--takeoff` | off | arm and climb both first |
| | `--rate` | 50 | loop rate, Hz |
| | `--t-max` | 180 | time limit, s |
| | `--capture` | 2 | capture radius, m |
| | `--vertical` | `hold` | `hold` = PD altitude hold; `law` = 3-D, law owns Down |
| | `--settle` | 3 | seconds flying the target before guidance |
| | `--status` | 1 | seconds between status lines; 0 silences |
| | `--on-finish` | `loiter` | `loiter`, `land`, `nothing` |
| | `-o` | | output CSV |
| target | `--vt` | 6 | target speed, m/s |
| | `--gam-t` | 180 | target heading, deg from North |
| | `--target` | `straight` | `straight`, `turn`, `weave` |
| | `--at` | 0 | target latax amplitude, m/s² |
| | `--weave-period` | 10 | s |

### `offline` — extra flags beyond the above

| flag | default | meaning |
|---|---|---|
| `--r0` | 500 | initial range, m |
| `--lam0` | 0 | initial LOS bearing, deg from North (**use 90 to match SITL**) |
| `--gam0` | 0 | initial pursuer heading, deg from North |
| `--lead-start` | off | start on the collision triangle |
| `--dt` | 0.01 | solver step, s |
| `--capture` | 1 | capture radius, m |

---

## 9. Things that will bite

- **Rebuild after editing SDF, `.parm` or launch files.** They install as copies; the
  installed copy is what runs.
- **Both SITL instances must share one `--custom-location`.** Different homes still fly and
  are quietly wrong. The launch file handles it.
- **The target starts 500 m EAST**, not North (measured: `lam = 90.9°` at t=0). The comment
  in `worlds/engagement.sdf` saying "North" is wrong. Offline equivalent: `--lam0 90`.
- **Guidance cannot create speed.** PN, pursuit and CLOS only steer an existing velocity
  vector. `--vp` exists because a SITL copter starts hovering at zero.
- **`Vp` in the logs is the 3-D speed norm**, so a climbing copter reads fast while going
  nowhere horizontally.
- **Keep `--a-max` and `ANGLE_MAX` consistent** between offline and SITL, or the comparison
  is meaningless. 30° → 5.66 m/s²; 45° → 9.81.
- **`config/sitl_guided.parm` is load-bearing.** It carries `FRAME_CLASS`/`FRAME_TYPE`,
  which ArduPilot 4.8-dev no longer supplies for `--model JSON`. Strip them and nothing
  arms.
- **The CSV is written at the end of a run**, not streamed — `tail -f` shows nothing. That
  is what `--status` is for.

### Changing the target's starting distance

Edit the `iris_target` include in `worlds/engagement.sdf`. `<pose>` is `X Y Z roll pitch
yaw` in the world **ENU** frame, and the plugin maps it to NED as `(N, E, D) = (y, x, −z)`:

```xml
<pose degrees="true">500 0 0.195 0 0 180</pose>
```

X (first) is **East** — that is the range knob. Y is North. Z = 0.195 is the iris's ground
clearance; leave it. The last number is heading in degrees.

- 800 m east: `800 0 0.195 0 0 180`
- 500 m north: `0 500 0.195 0 0 180`

Then rebuild, and match the offline geometry (`--r0`, `--lam0`). Do **not** change
`--custom-location` — both vehicles keep the same home.
