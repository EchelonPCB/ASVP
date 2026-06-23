#!/usr/bin/env python3
"""
T3 · 03_jetracer_node.py — ACDC JetRacer Pro · ROS 2 /cmd_vel -> PCA9685 bridge
================================================================================
The bridge from "ROS 2 runs" to "ROS 2 DRIVES the car." Standalone rclpy node —
NO colcon build needed, just run it with python3 inside the container.

Subscribes  /cmd_vel  (geometry_msgs/Twist):
    linear.x  > 0.05  -> ESC_FORWARD (creep)   ;  else -> ESC_NEUTRAL
    angular.z         -> steering, center + scaled, clamped to soft limits
Safety watchdog: if no /cmd_vel for 0.5 s, throttle drops to neutral.

Run (inside the container, ROS 2 sourced):
    python3 03_jetracer_node.py
Drive it from a SECOND shell into the same container:
    docker exec -it ros2_foxy bash
    source /opt/ros/foxy/setup.bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # (install once if missing: apt update && apt install -y ros-foxy-teleop-twist-keyboard)

WHEELS OFF THE GROUND for the first drive test.
If steering goes the WRONG way, set INVERT_STEER = True below.
"""
import sys
import time

try:
    import smbus2
except ImportError:
    sys.exit("smbus2 not found. Inside the container run:  pip3 install smbus2")

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── Authoritative ACDC hardware constants ──────────────────────────────────
BUS = 1
ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
CH_STEER = 0
CH_THROTTLE = 1
ESC_NEUTRAL = 307
ESC_FORWARD = 325
STEER_CENTER = 320
STEER_RIGHT_LIMIT = 280
STEER_LEFT_LIMIT = 370
# symmetric usable span so we never exceed either soft limit
STEER_SPAN = min(STEER_CENTER - STEER_RIGHT_LIMIT, STEER_LEFT_LIMIT - STEER_CENTER)

INVERT_STEER = False     # flip to True if the servo turns the wrong way
CMD_TIMEOUT_S = 0.5      # no cmd_vel within this -> failsafe neutral


class JetRacerNode(Node):
    def __init__(self):
        super().__init__('jetracer_node')
        self.bus = smbus2.SMBus(BUS)
        self._init_pca9685()
        self._set(CH_THROTTLE, ESC_NEUTRAL)
        self._set(CH_STEER, STEER_CENTER)
        self._last_cmd = time.monotonic()
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info('jetracer_node up — listening on /cmd_vel (wheels off ground!)')

    def _w(self, reg, val):
        self.bus.write_byte_data(ADDR, reg, val & 0xFF)

    def _r(self, reg):
        return self.bus.read_byte_data(ADDR, reg)

    def _set(self, ch, off):
        self.bus.write_i2c_block_data(ADDR, LED0_ON_L + 4 * ch, [0, 0, off & 0xFF, off >> 8])

    def _init_pca9685(self):
        self._w(MODE1, 0x00); time.sleep(0.1)
        self._w(MODE1, 0x10); time.sleep(0.1)
        self._w(PRESCALE, 121); time.sleep(0.1)
        self._w(MODE1, 0xA1); time.sleep(0.1)
        if self._r(PRESCALE) != 121:
            raise RuntimeError('PRESCALE readback wrong — check I2C bus 1 / 0x40')
        self.get_logger().info('PCA9685 verified (bus 1, 0x40, 50 Hz)')

    def _on_cmd(self, msg: Twist):
        self._last_cmd = time.monotonic()
        thr = ESC_FORWARD if msg.linear.x > 0.05 else ESC_NEUTRAL
        z = max(-1.0, min(1.0, msg.angular.z))
        if INVERT_STEER:
            z = -z
        steer = int(STEER_CENTER + z * STEER_SPAN)
        steer = max(STEER_RIGHT_LIMIT, min(STEER_LEFT_LIMIT, steer))
        self._set(CH_THROTTLE, thr)
        self._set(CH_STEER, steer)

    def _watchdog(self):
        if time.monotonic() - self._last_cmd > CMD_TIMEOUT_S:
            self._set(CH_THROTTLE, ESC_NEUTRAL)   # failsafe: stop, keep last steer

    def stop(self):
        try:
            self._set(CH_THROTTLE, ESC_NEUTRAL)
            self._set(CH_STEER, STEER_CENTER)
            time.sleep(0.2)
            self.bus.close()
        except Exception:
            pass


def main():
    rclpy.init()
    node = JetRacerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
