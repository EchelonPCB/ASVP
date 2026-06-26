# Restart runbook — Reg sign-off drive

## Where we are (one line)
Both Reg drive motors are bench-confirmed, the unified driver is code-complete and already on the Jetson, and the ROS 2 Foxy container is built. **You are one teleop drive away from signing off the Reg car.**

## Your setup
- Mac → SSH → Jetson. Claude → RDP on the Windows box.
- **The one rule:** prompt `jetson@…` = the Jetson **host** (outside Docker). Prompt `root@…` = **inside** the `ros2_foxy` container. Watch the prompt; that's your only signal.
- Get the Jetson IP if needed: on the Jetson run `hostname -I`.

## The resume sequence — the sign-off drive

**Terminal 1 (driver).** From your Mac:
```bash
ssh jetson@<JETSON_IP>
bash ~/Desktop/ASVP-main/docker/run_ros2_foxy.sh     # enters/re-enters the container → prompt becomes root@
# now inside (root@):
cd /ros2_ws/acdc_driver
grep ACTIVE car_config.py                            # must read jetracer_reg; if it says jetracer_pro:
sed -i 's/ACTIVE = "jetracer_pro"/ACTIVE = "jetracer_reg"/' car_config.py
python3 acdc_driver_node.py
#   expect: [INFO] acdc_driver up — platform 'JetRacer (regular / non-Pro)', drive 'dc_hbridge'
#   it then holds the terminal with no prompt — that's CORRECT, it's running. Leave it.
```

**Terminal 2 (keyboard, WASD).** Open a *second* Mac terminal, SSH in again:
```bash
ssh jetson@<JETSON_IP>
docker exec -it ros2_foxy bash                       # second shell into the SAME container
source /opt/ros/foxy/setup.bash
cd /ros2_ws/acdc_driver
python3 acdc_teleop_wasd.py
```

**WHEELS OFF GROUND.** WASD: `w` forward · `s` reverse · `a` steer right · `d` steer left · `z` stop + quit. Commands latch (held until you change them or hit `z`).

## If something trips
- `i` drives it **backward** → forward polarity is flipped; one-line fix in `DcHbridgeDrive` (`fwd` logic). Tell Claude.
- Driver errors `No module named 'rclpy'` → this shell didn't source ROS: `source /opt/ros/foxy/setup.bash`, re-run.
- `docker exec` says **no such container** → recreate it: `bash ~/Desktop/ASVP-main/docker/run_ros2_foxy.sh`.
- Packages missing (`smbus2` / teleop) → in the container: `apt update && apt install -y ros-foxy-teleop-twist-keyboard python3-pip && pip3 install smbus2`.
- Too slow / too fast on the ground → tune `DC_MAX` in `acdc_driver_node.py` (currently 1400).

## State & housekeeping
- The Jetson's existing `~/ros2_ws/acdc_driver` runs the Reg **driver** as-is, BUT the new **WASD teleop (`acdc_teleop_wasd.py`) is not on the Jetson yet** — it only exists locally until pushed.
- **Step 0 tomorrow (one-time, gets the teleop + fixes):** on the Jetson host —
  ```bash
  cd /tmp && rm -rf ASVP && git clone --depth 1 https://github.com/EchelonPCB/ASVP.git
  cp -r /tmp/ASVP/acdc_driver ~/ros2_ws/
  ```
  This only works **after Claude pushes**. Local-but-unpushed: `acdc_teleop_wasd.py`, `esc_forward 330` (Pro-only), spec actuation map / register fixes. Last pushed commit: `77199e5`.

## Hardware truth — do NOT let actuator and motor get swapped
Two separate systems per car:
- **Steering actuator (servo):** PCA9685 `0x40` · ch0 · reg `0x06` — both cars (Pro ticks 320/370/280; Reg 425/300/500 inverted).
- **Drive motor:** Pro = ESC on `0x40` · ch1 · reg `0x0A` (307/330/345). Reg = TB6612 on a **second** chip `0x60` — right pwm0/dir1,2, left pwm6/dir7,8.
- A register is meaningless without its chip address. Pro keeps both systems on `0x40`; Reg splits steering (`0x40`) from drive (`0x60`).

## After the sign-off drive
1. Sign off a **Pro** car the same way (`ACTIVE=jetracer_pro`, ESC path).
2. Confirm the open ❓ items: Pro power source, whether any car gets a LiDAR (gates SLAM/Nav2), Reg wheel encoders (present, unused).
3. Bake the custom `acdc-ros2:foxy` image and distribute fleet-wide (build once → `docker save | ssh | docker load`).
4. Multi-car: per-car ROS namespaces (`/car_03/cmd_vel`) + coordination.

## For a fresh Claude session
- Memory file `acdc-ros2-fleet-driver` loads automatically. Also read `acdc_driver/PLATFORM_SPEC.md` and `car_config.py`.
- Treat the **steering actuator** and the **drive motor** as two distinct systems per car; always carry the chip address with any channel/register.
