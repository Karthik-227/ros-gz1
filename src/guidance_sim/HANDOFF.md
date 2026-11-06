# Status and remaining verification

Steps 1-6 of the original plan are **complete**. See `README.md` for what the package does
and how to run it. This file records only what is and is not verified, so nobody has to
guess.

## Verified by execution

- All 8 guidance laws, the registry and `make_law()`
- The three analytic identities (PN≡ZEM at N=3, the PN/PPN scalar reductions, |ZEM⊥| = R²λ̇/Vc)
- Offline simulator: 40 runs across 5 engagement presets, all 8 laws
- Parameter sweep: 36 runs, law × N × speed ratio
- Both plot modes (run comparison, capturability chart)
- `colcon build` clean; all 6 console scripts resolve under `ros2 run`
- `flake8 --max-line-length 100` clean across `guidance_sim/`, `tools/`, `launch/`
- World and model SDF parse as valid XML; `fdm_port_in` is 9002 / 9012 as intended

## NOT verified — never executed against a live simulator

Written and syntax-checked, but no Gazebo or SITL process was ever started:

- `mavlink_io.py` — connection, arming, takeoff, the accel setpoint path
- `takeoff.py`, `engage.py`
- `worlds/engagement.sdf` loading in Gazebo
- `launch/engagement.launch.py`
- `config/sitl_guided.parm` actually loading

The MAVLink constants are confirmed against the ArduPilot source (type mask 3135 matches
the `pos_ignore && vel_ignore && !acc_ignore` branch; port stride 10 matches
`sim_vehicle.py:1932`), but reading the source is not the same as flying it.

## SITL bring-up checklist

Work through this in order; each step isolates one failure mode.

1. **World loads and both models appear.**
   ```bash
   cd /home/karthik/guidance_ws && source install/setup.bash && GZ_SIM_RESOURCE_PATH=$PWD/install/guidance_sim/share/guidance_sim/models:$GZ_SIM_RESOURCE_PATH gz sim -v4 -r install/guidance_sim/share/guidance_sim/worlds/engagement.sdf
   ```
   Expect `iris_pursuer` at the origin and `iris_target` 500 m North.

2. **One SITL instance connects to the plugin.**
   ```bash
   cd /home/karthik/guidance_ws/src/ardupilot && ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I 0 --custom-location=-35.363262,149.165237,584,353 --out=udp:127.0.0.1:14550 --no-rebuild --use-dir=/home/karthik/guidance_ws/sitl_state/pursuer
   ```
   If it hangs waiting for the FDM, the port in `models/iris_pursuer/model.sdf` is wrong.

3. **Second instance, same home.** As above with `-I 1` and `--out=udp:127.0.0.1:14560`.
   A different home here silently breaks the shared NED frame and every geometry number
   downstream — the engagement will look plausible and be wrong.

4. **Telemetry sanity before any guidance.** Confirm both `LOCAL_POSITION_NED` streams and
   that differencing them gives ~500 m with the target due North.

5. **Takeoff only.**
   ```bash
   cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff --url udp:127.0.0.1:14550 --alt 50
   ```

6. **Altitude hold alone.** Run `engage` with `--law pn --t-max 10` and watch whether the
   pursuer holds 50 m. If it sinks or climbs away, the sign or gains in
   `engage.altitude_hold()` are the first suspect, not the guidance law.

7. **Full engagement**, then overlay the SITL CSV against the offline run of the same
   geometry in `plot_run` and compare miss distance and peak latax.

## Known soft spots

- `engage.py` reads the target's velocity from telemetry but takes `a_t` from the
  *commanded* course. If the target copter lags its velocity setpoint, APN's feed-forward
  will be fed a slightly optimistic acceleration.
- The loop uses `time.sleep()` for pacing; under load the achieved rate will drop below
  50 Hz. The run prints the achieved rate — check it before trusting a result.
- Capture radius defaults to 2 m in SITL versus 1 m offline, because of position noise and
  finite control bandwidth. Miss distances below ~1 m are not meaningful on either path.
- `ARMING_CHECK 0` in `sitl_guided.parm` is a simulation convenience and has no business
  anywhere near hardware.
