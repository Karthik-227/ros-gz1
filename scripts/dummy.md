# Autonomous Drone Exploration — Run Guide

> **Which workspace is live?** `~/.bashrc` sources `~/ardu_ws`, not `~/ros2_ws`, so
> `ros2 launch` / `ros2 run` resolve packages out of `~/ardu_ws`. The maintained
> copy of this stack is `~/ardu_ws/src/nidar_sim`, which has proper launch files.
> The two `nidar_sim` checkouts are clones of the same repo whose histories have
> **diverged locally**, so `~/ros2_ws/src/nidar_sim/config/*.yaml` is an older
> file. Everything below points at the `~/ardu_ws` copies deliberately.

**Open a new terminal before your first run today** — `~/.bashrc` changed (Gazebo
resource paths) and an old shell will not have it.

---

## 0 · Always do this first

```bash
~/ardu_ws/scripts/nidar-clean.sh
```

**CHECK** it prints `--- survivors (should be empty) ---` with nothing under it.

Do not skip this and do not substitute `pkill -f`. Two full stacks running at
once is the most destructive state this project gets into, and it is easy to
reach: a second `iris_maze.launch.py` starts a second Gazebo whose SITL cannot
bind ports 5760/9002, leaving a drone on screen that no autopilot is driving,
two `ros_gz_bridge`es publishing `/clock`, and two `robot_state_publisher`s
fighting over `odom -> base_link`. The drone then appears offset in RViz and the
map judders, and nothing in any single node's log looks wrong.

> One gotcha: this script kills by matching command lines, so **do not run it
> from a shell whose own command line contains the launch string** — it will
> kill that shell too. Run it on its own.

---

## 1 · Simulation

```bash
ros2 launch ardupilot_gz_bringup iris_maze.launch.py lidar_dim:=2
```

**CHECK** the log contains `got segment base_link` … `got segment base_scan`,
and `ros2 topic hz /clock` is ~1000 Hz.

`lidar_dim:=2` is not optional — this is a **2D lidar, 2D map** stack
throughout. There is no 3D anywhere in it.

If the drone renders as a bare frame with no body or propellers, your shell
predates the `.bashrc` change. `ardupilot_gazebo`'s models reference their
meshes with `package://` URIs, which a plain `gz sim` cannot resolve — it needs
the ament **`share`** directory on `GZ_SIM_RESOURCE_PATH`, not just
`share/<pkg>/models`. Open a new terminal.

---

## 2 · Arm and take off

```bash
mavproxy.py --console --map --aircraft test --master=:14550
```

then, one at a time:

```
mode guided
arm throttle
takeoff 2
```

**CHECK** the drone is airborne in Gazebo at ~2 m before going further.

Take off **before** starting Nav2. A velocity setpoint overrides the takeoff
climb in GUIDED, so with the bridge already running `takeoff 2` arms, rises a
few centimetres and stops.

<details>
<summary>Scripted alternative (no MAVProxy prompt)</summary>

ArduPilot's DDS interface exposes the same three actions as ROS services, which
is handy for unattended runs:

```bash
ros2 service call /ap/v1/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
ros2 service call /ap/v1/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
ros2 service call /ap/v1/experimental/takeoff ardupilot_msgs/srv/Takeoff "{alt: 2.0}"
```

Arming can fail the first couple of attempts while the EKF settles — retry.
</details>

---

## 3 · Everything else, in one command

```bash
ros2 launch nidar_sim nidar_bringup.launch.py
```

That starts, in order: `scan_stabilizer` -> `slam_toolbox` -> Nav2 ->
`cmd_vel_bridge` -> `frontier_explorer`, with RViz preloaded. Add
`explore:=false` to stop before autonomy and drive it yourself with **2D Goal
Pose**; add `rviz:=false` to save CPU.

**CHECK**, in this order:

| Look for | From |
|---|---|
| `stabilised N scans \| mean tilt …` every 10 s | `scan_stabilizer` |
| `Registering sensor: [Custom Described Lidar]` | `slam_toolbox` |
| `Managed nodes are active` | `lifecycle_manager` |
| `Explored N m2 (X% of mapped area) \| … goal -> (x, y)` | `frontier_explorer` |

RViz should show **solid black walls** with a cyan inscribed band and a pink
inflation gradient.

<details>
<summary>Running the pieces separately</summary>

```bash
ros2 launch nidar_sim slam.launch.py          # stabilizer + slam_toolbox
ros2 launch nidar_sim nav2.launch.py          # Nav2 + RViz + cmd_vel bridge
ros2 run nidar_sim frontier_explorer --ros-args -p use_sim_time:=true
```

If you start the explorer by hand, `use_sim_time:=true` matters: without it
every timeout in the node is measured on the wall clock while the TF it reasons
about is stamped with Gazebo time, and Gazebo here runs at roughly half real
time. It now detects this and sets the parameter itself, but do not rely on it.
</details>

---

## How the perception layer works

`scan_stabilizer` republishes `/scan` as `/scan_safe`, and SLAM *and* both Nav2
costmaps read `/scan_safe`. It is 2D in, 2D out.

A 2D lidar reports (range, bearing) pairs and everything downstream reads them
as a horizontal slice of the world — only true while the sensor is level, and a
multirotor has to lean to accelerate, brake and turn. Leaned over, three things
go wrong at once: every return is filed under the wrong bearing (so a wall
smears across cells and never accumulates), the range is a slant distance
rather than a horizontal one, and — worst — a beam that skims *over* a near
wall and hits something beyond it makes SLAM ray-trace straight through that
wall and erase it.

The stabilizer re-projects every ray to its true horizontal bearing and range
using the airframe's current attitude, and emits **NaN** for any return it
cannot vouch for as horizontal — floor strikes above all. NaN is the important
part: SLAM and the costmaps both ignore it, so a withheld ray marks nothing
**and traces nothing**. It publishes in frame `base_scan_level` and broadcasts
`base_link -> base_scan_level` with roll and pitch removed.

Measured in flight: at 2–3 deg mean lean it passes **800–1060 of 1081 rays** per
scan, withholding almost nothing.

> `scan_tilt_gate` is the superseded node this replaced. It froze the scan
> stream while tilted, which starved slam_toolbox outright (a scan whose
> timestamp has not advanced is rejected) and could latch holding and never
> release. It still exists so an old command resolves, but it prints an error
> telling you not to use it.

---

## Stuck? Run these in order. **First silent one is the fault.**

```bash
ros2 topic hz /clock                                   # ~1000 Hz
ros2 topic info /clock                                 # exactly ONE publisher
ros2 topic hz /scan                                    # ~10 Hz, 1081 rays
ros2 run tf2_ros tf2_echo base_link base_scan          # lidar mount
ros2 run tf2_ros tf2_echo base_link base_scan_level    # stabilizer TF
ros2 topic hz /scan_safe                               # ~10 Hz
ros2 topic hz /map                                     # ~1 Hz
ros2 run tf2_ros tf2_echo map base_link                # SLAM localising
ros2 topic hz /cmd_vel                                 # only while moving
ros2 topic hz /ap/v1/cmd_vel                           # only while moving
```

| Silent | Cause |
|---|---|
| `/clock` | more than one Gazebo, or a bridge on a dead gz topic -> step 0 |
| `/scan` | wrong lidar — needs `lidar_dim:=2` |
| `base_link->base_scan` | `robot_state_publisher` died -> reapply `patches/` |
| `base_link->base_scan_level` | stabilizer has no attitude source; it logs `NOT STABILISING` |
| `/scan_safe` | stabilizer not running -> use the launch files, not raw slam_toolbox |
| `/map` | SLAM has no scan — check the two above |
| `map->base_link` | SLAM not localising |
| `/cmd_vel` | Nav2 not active, or no goal |
| `/ap/v1/cmd_vel` | bridge not running -> `ros2 run nidar_sim cmd_vel_bridge` |

---

## Symptom -> cause

### Walls are not solid black in the map

| What you see | Cause | Where |
|---|---|---|
| Black dots with grey gaps, worse further from the drone | lidar angular resolution too coarse at range | `samples` in `lidar_2d/model.sdf` (1081) |
| White "fans" radiating into grey with no black at the end | returns past `max_laser_range` traced as free but never marked | `max_laser_range` in `mapper_params.yaml` (8.0) |
| A black wall turns white as the drone flies past | tilted beams skimming over it and ray-tracing through | `scan_stabilizer` must be running |

In the **costmap**, if there is no grey anywhere, `track_unknown_space` is off
and every cell the lidar has never seen reads as confirmed-free — the planner
will route straight through unexplored space. Both that and
`planner_server.allow_unknown` are set correctly in `config/nav2_params.yaml`.

### The drone hits a wall and everything stops

Look in the MAVProxy console for:

```
AP: Crash: Disarming: AngErr=38>30, Accel=0.1<3.0
```

That is **ArduPilot's own crash check**, not Nav2. It is the signature of a
multirotor pinned against something while still being commanded to move: the
motors cut, the drone drops and tips over, and afterwards it reads as a large
constant tilt (60–65 deg) lying on the ground. Everything downstream then looks
broken — the explorer reports `STUCK: moved 0.00 m`, the stabilizer withholds
most rays because the airframe really is on its side — but all of that is
*downstream of the disarm*, not the cause.

Guards now in place, in order of how early they act:

1. RPP forward-simulates **1.5 s** ahead for collisions
   (`max_allowed_time_to_collision_up_to_carrot`), sized for the lag between
   Nav2 deciding to stop and thrust actually changing.
2. `velocity_smoother.velocity_timeout: 0.25` ramps to zero quickly when
   `controller_server` aborts.
3. `cmd_vel_bridge`'s deadman (`command_timeout_sec: 0.3`) sends a stop if
   `/cmd_vel` goes **silent** while the drone is still moving. This is the one
   that matters most: ArduPilot holds the last commanded velocity for ~3 s, so
   before this existed a Nav2 abort meant the drone carried on at cruise speed
   into whatever the controller had just refused to enter.

If it still happens, lower `desired_linear_vel` (currently 0.6) before touching
anything else.

### "It's drifting"

Measure before tuning. Over an instrumented flight, `map -> odom` drifted
**0.095 deg of yaw and 0.137 m**, with no yaw jump above 0.5 deg.
`odom -> base_link` comes from Gazebo's odometry plugin with no noise
configured — it is ground truth — so `map -> odom` staying near identity is
correct and SLAM is doing its job. If the drone appears to wander, check
altitude and arm state first: a sinking or disarmed drone looks exactly like
drift.

Altitude hold is solid, and that was measured too — 2.000 m held flat across
9 m of commanded flight.

### The map strobes / flickers in RViz

Should be gone: `always_send_full_costmap` is now `false`, so Nav2 publishes
only the rectangle that changed instead of all ~235k cells at 1.5 Hz. If it
returns, that parameter is the first thing to check.

---

## Tuning for coverage

| Symptom | Parameter | Try |
|---|---|---|
| Ignores far rooms | `distance_decay_m` | 4.0 -> **6.0** |
| Wanders, skips nearby | `distance_decay_m` | 4.0 -> **2.5** |
| Still oscillates | `direction_weight_m` | 2.5 -> **4.0** |
| Misses narrow doorways | `min_frontier_cells` | 8 -> **5** |
| Stops too early | `empty_streak_limit` | 5 -> **8** |
| Goals near walls get rejected | `safe_distance_m` | keep >= `robot_radius` (0.35) |

```bash
ros2 run nidar_sim frontier_explorer --ros-args \
  -p use_sim_time:=true -p distance_decay_m:=6.0 -p min_frontier_cells:=5
```

---

## Never do

- Source two workspaces -> two `navigate_to_pose` servers racing your goals
- Start a second `iris_maze.launch.py` -> two Gazebos, two `/clock` publishers
- Run raw `slam_toolbox online_async_launch.py` -> no stabilizer -> no `/scan_safe` -> no map
- Run `scan_tilt_gate` -> deprecated; it freezes the scan stream and starves SLAM
- Run `topic_relay_node` **and** `cmd_vel_bridge` -> two publishers on `/ap/v1/cmd_vel`
- `git pull` in `src/ardupilot_gz` without reapplying `patches/`
