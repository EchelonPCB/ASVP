#!/usr/bin/env bash
# T0 — ROS 2 Foxy sanity. Run INSIDE the ROS 2 container.
echo "==== T0 · ROS 2 Foxy container check ===="
source /opt/ros/foxy/setup.bash 2>/dev/null || { echo "FAIL: cannot source ROS 2 Foxy"; exit 1; }
echo "ROS_DISTRO     = ${ROS_DISTRO}"
echo "ROS_DOMAIN_ID  = ${ROS_DOMAIN_ID:-<UNSET — set the SAME value on every car!>}"
echo "RMW            = ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (default)}"
echo "--- ros2 topic list ---"
ros2 topic list
echo "-----------------------"
echo "PASS if you see at least /parameter_events and /rosout above."
