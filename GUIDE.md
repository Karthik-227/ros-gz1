# guidance_ws — commands

## 0. Every new terminal

```bash
cd /home/karthik/guidance_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```

## 1. Rebuild (after any edit)

```bash
cd /home/karthik/guidance_ws && colcon build --packages-select guidance_sim --symlink-install
```

## 2. Terminal 1 — simulator

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 launch guidance_sim engagement.launch.py
```

Wait for `AP: ArduPilot Ready` twice.

## 3. Terminal 2 — take off both

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim takeoff_both --alt 5
```

## 4. Terminal 3 — run one engagement

### 3a. PP — pure pursuit

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pure_pursuit -k 2 --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_pp.csv
```

### 3b. DPP — deviated pure pursuit

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law deviated_pursuit -k 2 --delta 20 --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_dpp.csv
```

### 3c. LOSG — line of sight guidance

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law los --kp 1.0 --kd 2.5 --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_losg.csv
```

### 3d. PN — proportional navigation

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law pn -N 3 --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_pn.csv
```

### 3e. PPN — pure proportional navigation

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law ppn -N 3 --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_ppn.csv
```

### 3f. TPN — true proportional navigation

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim engage --law tpn -N 3 --vp 12 --alt 50 --vt 6 --gam-t 180 -o runs/sitl_tpn.csv
```

`tpn` and `pn` are the same law — identical output.

### AFTER CHANGING GEOMETRY under /src, always rebuild to applyb changes
```
cd /home/karthik/guidance_ws && colcon build --packages-select guidance_sim --symlink-install
```

## 5. Terminal 2 — land

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim land_both
```

## 6. Plots

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/sitl_pn.csv --show
```

```bash
cd /home/karthik/guidance_ws && source install/setup.bash && ros2 run guidance_sim plot_run runs/sitl_pp.csv runs/sitl_pn.csv runs/sitl_ppn.csv runs/sitl_losg.csv -o runs/laws_compare.png --show
```

## 7. Restart clean

```bash
pkill -f 'arducopter|sim_vehicle.py|gz sim' ; sleep 3 ; pgrep -af 'arducopter|gz sim' || echo "all clear"
```

## 8. Reset stored SITL parameters

```bash
cd /home/karthik/guidance_ws && rm -f sitl_state/pursuer/eeprom.bin sitl_state/target/eeprom.bin
```

## 9. Check both models loaded

```bash
gz model --list
```

---

Setup from scratch: [SETUP.md](SETUP.md) · Full reference: [COMMANDS.md](COMMANDS.md) ·
Troubleshooting: [SETUP.md](SETUP.md) §12 · Architecture: [WORKSPACE_HANDOFF.md](WORKSPACE_HANDOFF.md)