#!/usr/bin/env bash
# ============================================================================
# setup_jetson_docker_ros2.sh
# One-time host setup to run ROS 2 Foxy in Docker on a Jetson Nano
# (JetPack 4.x / Ubuntu 18.04 / Python 3.6 host).
#
# Idempotent — safe to re-run. Run on EACH car:
#     bash setup_jetson_docker_ros2.sh
#
# Uses `sudo docker` because on first run you are not yet in the docker group
# (that only takes effect after you log out/in). After re-login you can drop sudo.
# ============================================================================
set -e

echo "==== ACDC Jetson · Docker + ROS 2 Foxy host setup ===="

# --- 0. Environment report -------------------------------------------------
echo "[0] Environment:"
head -1 /etc/nv_tegra_release 2>/dev/null || echo "    (no /etc/nv_tegra_release found)"
echo -n "    python3 : "; python3 --version 2>&1
echo -n "    hostname: "; hostname
echo    "    disk    : $(df -h / | awk 'NR==2{print $4" free of "$2}')"
echo    "    memory  : $(free -h | awk '/Mem/{print $2" RAM"} /Swap/{print $2" swap"}' | paste -sd' ' -)"

# --- 1. Docker present? (JetPack usually ships it) -------------------------
if command -v docker >/dev/null 2>&1; then
  echo "[1] Docker already installed: $(docker --version)"
else
  echo "[1] Docker not found — installing nvidia-docker2 (Jetson-appropriate)..."
  sudo apt-get update
  sudo apt-get install -y nvidia-docker2
fi

# --- 2. Add user to docker group ------------------------------------------
if id -nG "$USER" | tr ' ' '\n' | grep -qx docker; then
  echo "[2] $USER already in docker group"
else
  echo "[2] Adding $USER to docker group (LOG OUT/IN after this script to use docker without sudo)"
  sudo usermod -aG docker "$USER"
fi

# --- 3. Set nvidia as default runtime -------------------------------------
DAEMON=/etc/docker/daemon.json
echo "[3] Writing $DAEMON (nvidia default runtime)"
if [ -f "$DAEMON" ]; then sudo cp "$DAEMON" "${DAEMON}.bak.$(date +%s)"; fi
sudo tee "$DAEMON" >/dev/null <<'JSON'
{
  "default-runtime": "nvidia",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  }
}
JSON
sudo systemctl restart docker
echo -n "    "; sudo docker info 2>/dev/null | grep -i "Default Runtime" || echo "(could not read runtime — check 'sudo docker info')"

# --- 4. Max performance mode ----------------------------------------------
echo "[4] Setting 10W MAXN + jetson_clocks"
sudo nvpmodel -m 0 || true
sudo jetson_clocks || true

# --- 5. Swap (4 GB) — critical on the 4 GB Nano ---------------------------
if swapon --show 2>/dev/null | grep -q '/swapfile'; then
  echo "[5] Swapfile already active"
else
  echo "[5] Creating 4 GB swapfile"
  sudo fallocate -l 4G /swapfile 2>/dev/null || sudo dd if=/dev/zero of=/swapfile bs=1M count=4096
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  grep -q '/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
fi

# --- 6. Pull ROS 2 Foxy image ---------------------------------------------
echo "[6] Pulling ros:foxy-ros-base-focal (~600 MB, one time)..."
sudo docker pull ros:foxy-ros-base-focal

mkdir -p "$HOME/ros2_ws/src"

echo ""
echo "==== DONE ===================================================="
echo "If step 2 added you to the docker group, LOG OUT AND BACK IN now"
echo "(or run:  newgrp docker ) so you can use docker without sudo."
echo ""
echo "Smoke-test ROS 2 Foxy:"
echo "  docker run -it --rm --network host ros:foxy-ros-base-focal \\"
echo "    bash -lc 'source /opt/ros/foxy/setup.bash && ros2 topic list && echo ROS2_FOXY_OK'"
echo ""
echo "Then launch the working container:  bash run_ros2_foxy.sh"
echo "============================================================="
