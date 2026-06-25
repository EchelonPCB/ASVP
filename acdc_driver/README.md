# acdc_driver — ACDC fleet ROS 2 car driver

The production `/cmd_vel` → car driver for the fleet. **One image + one node +
a per-platform profile.** This is the successor to
`ros2_jetson_test/03_jetracer_node.py` — same proven PCA9685 sequence, now
profile-driven and multi-platform.

| File | What it is |
|---|---|
| `car_config.py` | The platform profiles — the single source of truth (Pro ✅ / Reg ⚠️ on hold) |
| `acdc_driver_node.py` | The universal ROS 2 node; reads the active profile, branches on drive type |
| `PLATFORM_SPEC.md` | The authoritative Pro-vs-Reg spec (keep in sync with `car_config.py`) |

## Run on a JetRacer **Pro** (works today)

This rides on the Docker scaffold in `../docker/`. The container mounts
`$HOME/ros2_ws` → `/ros2_ws`, so put this folder there.

```bash
# on the car (host): get the code into the mounted workspace
cp -r acdc_driver ~/ros2_ws/

# start / re-enter the persistent Foxy container
bash ~/docker/run_ros2_foxy.sh

# inside the container:
pip3 install smbus2                                   # if not already in the image
cd /ros2_ws/acdc_driver
ACDC_CAR=jetracer_pro python3 acdc_driver_node.py     # <- "ROS 2 DRIVES the car"
#   -> [INFO] acdc_driver up — platform 'JetRacer Pro', drive 'esc' ...

# second shell from the host, to drive it:
docker exec -it ros2_foxy bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**WHEELS OFF THE GROUND for the first drive test.** Throttle is gentle creep
(`linear.x > 0.05` → ESC 325); no command for 0.5 s → ESC neutral (failsafe).

## Flash to the other Pro cars

Nothing per-car changes. Push the *same image* (see `../docker/README.md` §6 —
`docker save | ssh | docker load`), drop this *same folder* in `~/ros2_ws`, and
run with `ACDC_CAR=jetracer_pro`. Identical across all 6.

For multi-car coordination, give each car a unique ROS namespace (e.g. remap
`/cmd_vel` → `/car_03/cmd_vel`); the hardware profile stays identical.

## JetRacer **Reg** — live (needs a test drive)

Both drive motors are bench-confirmed on 0x60 — right `pwm0 dir1,2`, left
`pwm6 dir7,8` — `DcHbridgeDrive` is implemented and the profile is
`validated=True`. Run it like the Pro, with the Reg profile:

```bash
cd /ros2_ws/acdc_driver
ACDC_CAR=jetracer_reg python3 acdc_driver_node.py
```

`linear.x` drives both rear motors together (Ackermann — the servo steers);
`DC_MAX` in `DcHbridgeDrive` tunes ground speed. **WHEELS OFF GROUND** for the
first `/cmd_vel` test — and eyeball direction: if `linear.x > 0` drives it
*backward*, flip the `fwd` polarity (one line) in `DcHbridgeDrive`.

See `PLATFORM_SPEC.md` for the full Pro-vs-Reg breakdown.
