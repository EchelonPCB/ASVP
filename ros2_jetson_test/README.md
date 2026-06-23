# ROS 2 Jetson Test — Operating Procedure

A 5-step ladder to prove the Docker + ROS 2 Foxy setup works **with your real
hardware**, one layer at a time. Each step has a clear PASS criterion. If a step
fails, stop there — that's the layer to fix.

```
T0  ROS 2 Foxy runs in the container
T1  the container can SEE the PCA9685 (I2C bus 1)
T2  the container can DRIVE the PCA9685 (steering + ESC)   <-- the real test
T3  ROS 2 drives the car via /cmd_vel (keyboard teleop)
T4  two cars see each other over DDS (fleet networking)
```

---

## ⚠️ Safety
- **Wheels OFF the ground** for any throttle/ESC test (T2 `--throttle`, T3).
- Every script returns the ESC to **neutral (307)** and re-centers steering on
  exit / Ctrl+C. T3 also has a 0.5 s **failsafe** that cuts throttle if commands stop.
- These use your **authoritative constants** (bus 1, 0x40, 50 Hz, STEER_CENTER 320,
  limits 280–370, ESC_NEUTRAL 307, ESC_FORWARD 325) — not generic guide values.

## Prerequisites
- Docker + ROS 2 Foxy already set up (see `../docker/`). 
- The car is on the **router LAN** (not the phone hotspot).
- For T4 you need a second car set up the same way.

---

## Step A — Get the tests onto the Jetson
Put them under `~/ros2_ws/` so they mount into the container at `/ros2_ws/`:
```bash
# from your PC, in the JPA folder
scp -r ros2_jetson_test jetson@<CAR_IP>:~/ros2_ws/
ssh jetson@<CAR_IP> "sed -i 's/\r$//' ~/ros2_ws/ros2_jetson_test/*.sh ~/ros2_ws/ros2_jetson_test/*.py"
```

## Step B — Enter the container
```bash
ssh jetson@<CAR_IP>
bash ~/docker/run_ros2_foxy.sh            # or: docker start -i ros2_foxy
# now you are INSIDE the container; the tests are at /ros2_ws/ros2_jetson_test
cd /ros2_ws/ros2_jetson_test
```

## Step C — One-time test deps (skip if you built the fleet image `acdc-ros2:foxy`)
```bash
apt-get update && apt-get install -y i2c-tools ros-foxy-teleop-twist-keyboard
pip3 install smbus2
```

---

## The Ladder

### T0 — ROS 2 Foxy runs
```bash
bash 00_container_check.sh
```
**PASS:** prints `ROS_DISTRO = foxy` and a topic list with `/rosout`,
`/parameter_events`. Note your `ROS_DOMAIN_ID` — it must match on every car.

### T1 — Container sees the PCA9685
```bash
bash 01_i2c_check.sh
```
**PASS:** `40` appears in the `i2cdetect` grid for bus 1.
**FAIL → fix:** if `/dev/i2c-*` is missing, the container lacks device access —
relaunch with `--privileged -v /dev:/dev` (the run script already does this).

### T2 — Container DRIVES the hardware  ← the test that matters
```bash
python3 02_pwm_test.py              # steering sweep only (safe, wheels can be down)
python3 02_pwm_test.py --throttle   # adds a brief ESC spin — WHEELS OFF GROUND
```
**PASS:** wheels turn right → left → re-center. With `--throttle`, the motor
creeps forward ~1.5 s then stops. **This proves Docker → I2C → motors works.**
**FAIL → fix:** PRESCALE error = wrong bus/wiring; no ESC spin = ESC needs its
power-on arming (power-cycle the ESC at neutral, re-run).

### T3 — ROS 2 drives the car
Shell 1 (the node):
```bash
python3 03_jetracer_node.py
```
Shell 2 (a second shell into the same container, from the host):
```bash
docker exec -it ros2_foxy bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Drive with the `i / , / j / l` keys (wheels off ground).
**PASS:** steering + throttle respond to keys; throttle cuts when you stop typing.
**FAIL → fix:** steering reversed? set `INVERT_STEER = True` in `03_jetracer_node.py`.
No topic seen? confirm both shells are the same container and ROS 2 is sourced.

### T4 — Two cars see each other (DDS)
Make sure **both** cars have the **same `ROS_DOMAIN_ID`** (set in `run_ros2_foxy.sh`).
On **car01** (in container): `ros2 run demo_nodes_cpp talker`
On **car02** (in container): `ros2 run demo_nodes_cpp listener`
*(install once if missing: `apt install -y ros-foxy-demo-nodes-cpp`)*
**PASS:** car02 prints `I heard: [Hello World: N]`. Fleet networking works.
**FAIL → fix:** different domain IDs, not on the same router LAN, or the container
wasn't started with `--network host`.

---

## What success means
- **T0–T2 green** = your stated goal is fully met: Docker + ROS 2 Foxy running
  *and* able to drive the car from inside the container.
- **T3 green** = you can now drive any car from ROS 2 (`/cmd_vel`) — the hook Nav2
  and the multi-vehicle coordination layer plug into.
- **T4 green** = the fleet can talk; ready for multi-car work.

## Notes
- **Camera not tested here on purpose** — your CSI perception needs `nvargus`,
  which isn't in the base Foxy image (see `../docker/README.md`). These tests are
  the control + networking layers; perception is a separate integration.
- `03_jetracer_node.py` is a **standalone** node (no colcon needed). When you're
  ready to make it a proper package + launch file, say the word.

## Report back
Paste the output of **T0, T1, and T2** and I'll confirm you're green or pinpoint
the fix.
