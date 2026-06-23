# ============================================================================
# Dockerfile — ACDC fleet ROS 2 Foxy image (build ONCE, distribute to all cars)
#
# Base: official ROS 2 Foxy (Ubuntu 20.04 / Python 3.8 inside the container —
#       which RETIRES the host's Python 3.6 backport pain).
#
# Includes your car-control deps (smbus2 for the PCA9685 on I2C bus 1) plus the
# Nav2 / SLAM / LiDAR stack for the "brain" car.
#
# NOTE ON THE CAMERA: this base image does NOT contain the Jetson multimedia
# stack (nvargus), so your CSI-camera perception (nvarguscamerasrc) will NOT
# run in here. For perception in-container, either (a) use a USB camera, or
# (b) switch the base to a dusty-nv l4t image
#     (e.g. dustynv/ros:foxy-ros-base-l4t-r32.7.1). See README.md.
#
# Build (on one car):   docker build -t acdc-ros2:foxy -f Dockerfile .
# ============================================================================
FROM ros:foxy-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-smbus i2c-tools \
      ros-foxy-navigation2 ros-foxy-nav2-bringup \
      ros-foxy-slam-toolbox ros-foxy-robot-localization \
      ros-foxy-teleop-twist-keyboard ros-foxy-joy \
      ros-foxy-cv-bridge ros-foxy-image-transport \
      ros-foxy-rplidar-ros \
      python3-colcon-common-extensions \
      nano \
    && rm -rf /var/lib/apt/lists/*

# Python deps for the car-control node (matches your demoday.py: smbus2 on bus 1)
RUN pip3 install --no-cache-dir smbus2 numpy

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc \
 && echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

WORKDIR /ros2_ws
