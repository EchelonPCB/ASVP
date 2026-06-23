# Docker + ROS 2 Foxy on the ACDC Jetson Nano Fleet

Goal: get Docker running and **ROS 2 Foxy** up on each Jetson Nano (JetPack 4.x /
Ubuntu 18.04). Ubuntu 18.04 can't install Foxy natively, so we run it in a
container — the container is Ubuntu 20.04 / Python 3.8, which also retires the
host's Python 3.6 backport pain.

> **These run ON the Jetson** (it's a Linux box). From your Mac/PC, SSH in first.
> Two ways to use this folder: SCP the scripts over, **or** just copy-paste the
> command blocks below into the SSH session.

---

## 0. Connect (on the router LAN now, not the hotspot)
```bash
# from your computer
ssh jetson@<CAR_IP>          # password: jetson   (get IP on the Jetson: hostname -I)
```

## 1. Copy these scripts to the Jetson (optional — or paste commands directly)
```bash
# from your computer, in the JPA folder
scp -r docker jetson@<CAR_IP>:~/
# on the Jetson, strip Windows line-endings just in case:
ssh jetson@<CAR_IP> "sed -i 's/\r$//' ~/docker/*.sh"
```

## 2. One-time host setup (per car)
```bash
bash ~/docker/setup_jetson_docker_ros2.sh
```
This is idempotent. It: reports the environment, ensures Docker + the nvidia
runtime, adds you to the docker group, sets 10W MAXN, creates a 4 GB swapfile
(critical on 4 GB Nano), and pulls `ros:foxy-ros-base-focal`.
**Then log out and back in** (or `newgrp docker`) so docker works without sudo.

## 3. Prove ROS 2 Foxy runs (30-second smoke test)
```bash
docker run -it --rm --network host ros:foxy-ros-base-focal \
  bash -lc 'source /opt/ros/foxy/setup.bash && ros2 topic list && echo ROS2_FOXY_OK'
```
If you see a topic list and `ROS2_FOXY_OK`, **you're done with the stated goal.**

## 4. Launch the working (persistent) container
```bash
bash ~/docker/run_ros2_foxy.sh
# inside:  ros2 topic list
# extra shell from host:  docker exec -it ros2_foxy bash
```
`--privileged -v /dev:/dev` gives the container your **I2C bus 1** (PCA9685
steering/ESC) and **USB** (LiDAR). `--network host` + a shared `ROS_DOMAIN_ID`
(=42 in the script) is what lets the cars see each other.

---

## 5. Two-car DDS check (the thing that failed on the hotspot)
On **car01** (inside the container): `ros2 run demo_nodes_cpp talker`
On **car02** (inside the container): `ros2 run demo_nodes_cpp listener`
If car02 prints "I heard: Hello World", DDS discovery works across the router LAN.
*(needs `ros-foxy-demo-nodes-cpp` — it's in the fleet image below, or `apt install` it.)*

## 6. Fleet: build ONCE, push to all 6 (don't repeat setup 6×)
On one car, build the custom image (adds Nav2/SLAM/LiDAR + smbus2):
```bash
cd ~/docker && docker build -t acdc-ros2:foxy -f Dockerfile .
```
Distribute the identical image to the others over SSH:
```bash
for ip in <CAR02_IP> <CAR03_IP> <CAR04_IP> <CAR05_IP> <CAR06_IP>; do
  docker save acdc-ros2:foxy | gzip | ssh jetson@$ip 'gunzip | docker load'
done
```
Then point `run_ros2_foxy.sh`'s `IMAGE=` at `acdc-ros2:foxy`.

---

## ⚠️ The camera caveat (read before wiring perception)
The official `ros:foxy-ros-base-focal` image has **no Jetson multimedia stack**,
so your CSI-camera perception (`nvarguscamerasrc` in `demoday.py` /
`lane_detect.py` / `traffic_light_detector.py`) **will not run inside it.**
This is fine for "get ROS 2 Foxy on" + DDS + Nav2 + LiDAR. When you need camera
perception in a container, pick one:
- **USB camera** — works via standard V4L2/OpenCV in any container (simplest).
- **Switch the base image** to `dustynv/ros:foxy-ros-base-l4t-r32.7.1` (has the
  multimedia stack + CUDA; bigger image, built by NVIDIA's Dustin Franklin).
- Mount the argus socket + libs into the official image (fiddly; last resort).

You can also keep perception running **natively on the host (no Docker)** as you
do today, and only use the container for the ROS 2 / Nav2 / coordination layer —
bridging the two over UDP or a ROS 2 topic. Often the least-friction path.

## Your hardware constants (use these, not generic guide values)
I2C **bus 1** (bus 0 dead) · PCA9685 **0x40** · CH0 steering / CH1 throttle ·
prescale **121 → 50 Hz** · STEER_CENTER **320** (limits 280–370) ·
ESC_NEUTRAL **307** · ESC_FORWARD ~**325–330**.

## Troubleshooting
| Symptom | Fix |
|---|---|
| `docker: permission denied` | You haven't re-logged-in after the group add. `newgrp docker` or log out/in. |
| Killed / OOM during pull or build | Swap not active. Re-run setup step 5; confirm with `free -h`. |
| `nvidia-smi: not found` | Normal on Jetson (integrated SoC). Use `tegrastats` or `sudo pip3 install jetson-stats && jtop`. |
| Cars can't see each other's topics | Different `ROS_DOMAIN_ID`, or not on the same router LAN, or not `--network host`. |
| `\r: command not found` running a script | Windows line-endings. `sed -i 's/\r$//' *.sh`. |
| Camera won't open in container | Expected — see the camera caveat above. |
