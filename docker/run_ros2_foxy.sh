#!/usr/bin/env bash
# ============================================================================
# run_ros2_foxy.sh — launch / re-enter the persistent ROS 2 Foxy container.
# Run AFTER setup_jetson_docker_ros2.sh and after you've logged out/in.
#     bash run_ros2_foxy.sh
#
# Flags explained:
#   --runtime nvidia   GPU + L4T libs (harmless for CPU-only ROS, needed later)
#   --network host     ROS 2 / DDS discovery across the fleet (the router LAN)
#   --privileged -v /dev:/dev   I2C (PCA9685 steering/ESC on bus 1) + USB (LiDAR)
#   -v ros2_ws         your code + logs persist on the host, survive container rm
#   -e ROS_DOMAIN_ID   MUST be identical on every car so they see each other
# ============================================================================
set -e
NAME=ros2_foxy
IMAGE=ros:foxy-ros-base-focal
DOMAIN_ID=42          # <-- same value on ALL cars

# Re-enter if the container already exists
if docker ps -a --format '{{.Names}}' | grep -qx "${NAME}"; then
  echo "[*] Container '${NAME}' exists — re-entering."
  echo "    (detach without stopping: Ctrl-P then Ctrl-Q   |   stop: type 'exit')"
  docker start -i "${NAME}"
  exit 0
fi

echo "[*] Creating container '${NAME}' from ${IMAGE} (ROS_DOMAIN_ID=${DOMAIN_ID})"
docker run -it \
  --runtime nvidia \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v "$HOME/ros2_ws":/ros2_ws \
  -w /ros2_ws \
  -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
  --name "${NAME}" \
  "${IMAGE}" \
  bash -lc '
    grep -q "source /opt/ros/foxy/setup.bash" /root/.bashrc 2>/dev/null \
      || echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
    echo "export ROS_DOMAIN_ID='"${DOMAIN_ID}"'" >> /root/.bashrc
    source /opt/ros/foxy/setup.bash
    echo ""
    echo "ROS 2 Foxy ready.  Verify with:"
    echo "   ros2 topic list"
    echo "   ros2 run demo_nodes_cpp talker      # (needs ros-foxy-demo-nodes-cpp)"
    echo "Open another shell into this container from the host:"
    echo "   docker exec -it '"${NAME}"' bash"
    exec bash
  '
