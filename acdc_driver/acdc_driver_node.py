#!/usr/bin/env python3
"""
acdc_driver_node.py — ACDC fleet universal /cmd_vel -> car driver (ROS 2 Foxy)
==============================================================================
ONE node for the whole fleet. It loads a PLATFORM profile from car_config.py and
drives the car accordingly, so the Docker image and this node are IDENTICAL on
every car — the only thing that changes per platform is the profile.

  drive = "esc"        -> JetRacer PRO : ESC pulse on PCA9685       (VALIDATED, live)
  drive = "dc_hbridge" -> JetRacer REG : DC motors via H-bridge @0x60 (ON HOLD —
                          left motor not bench-validated; throttle stays DISABLED,
                          steering still works, until you sign it off)

Pick the platform (env var beats car_config.ACTIVE):
    ACDC_CAR=jetracer_pro python3 acdc_driver_node.py

Subscribes /cmd_vel (geometry_msgs/Twist):
    linear.x  -> throttle (platform-specific backend)
    angular.z -> steering (PCA9685; +z = LEFT per REP-103; profile center/limits)
Safety: no /cmd_vel within CMD_TIMEOUT_S -> throttle to neutral (steer holds).
        *** WHEELS OFF THE GROUND for first drive tests. ***

Production successor to ros2_jetson_test/03_jetracer_node.py — same proven
PCA9685 sequence, now profile-driven and multi-platform. Run it co-located with
car_config.py (same folder).
"""
import os
import sys
import time

try:
    import smbus2
except ImportError:
    sys.exit("smbus2 not found. Inside the container:  pip3 install smbus2")

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import car_config

# ── PCA9685 registers ────────────────────────────────────────────────────────
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
PRESCALE_50HZ = 121          # 25 MHz / (4096 * 50 Hz) - 1
CMD_TIMEOUT_S = 0.5


class Pca9685:
    """Minimal raw-smbus2 PCA9685 at 50 Hz, verified by prescale readback."""
    def __init__(self, bus, addr):
        self.addr = addr
        self.bus = smbus2.SMBus(bus)
        self._init()

    def _w(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def _r(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def _init(self):
        self._w(MODE1, 0x00); time.sleep(0.1)
        self._w(MODE1, 0x10); time.sleep(0.1)           # sleep to set prescale
        self._w(PRESCALE, PRESCALE_50HZ); time.sleep(0.1)
        self._w(MODE1, 0xA1); time.sleep(0.1)           # wake + auto-inc + allcall
        if self._r(PRESCALE) != PRESCALE_50HZ:
            raise RuntimeError(
                f"PCA9685 @ 0x{self.addr:02X}: prescale readback wrong — wrong bus/addr?")

    def set(self, ch, off):
        self.bus.write_i2c_block_data(
            self.addr, LED0_ON_L + 4 * ch, [0, 0, off & 0xFF, off >> 8])

    def digital(self, ch, high):
        """Near-digital level for H-bridge direction pins."""
        self.set(ch, 4095 if high else 0)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


# ── Drive backends (the platform-specific part) ──────────────────────────────
class EscDrive:
    """JetRacer PRO: single ESC pulse on a PCA9685 channel. VALIDATED."""
    def __init__(self, pca, ch, neutral, forward):
        self.pca, self.ch = pca, ch
        self.neutral, self.forward = neutral, forward

    def command(self, linear_x):
        self.pca.set(self.ch, self.forward if linear_x > 0.05 else self.neutral)

    def neutral_stop(self):
        self.pca.set(self.ch, self.neutral)


class NullDrive:
    """Throttle disabled (platform on hold / not validated). Steering still works."""
    def command(self, linear_x):
        pass

    def neutral_stop(self):
        pass


class DcHbridgeDrive:
    """JetRacer REG: 2 DC gearmotors via TB6612 H-bridge on a PCA9685 @ 0x60.

    Ackermann car (the servo on 0x40 steers), so both drive motors run the SAME
    direction + speed from linear.x. Bench-confirmed channels:
        right (Motor A): pwm0, dir 1/2     left (Motor B): pwm6, dir 7/8
    Both motors read "backwards then forwards" under the discovery tool, so real
    FORWARD = dir_a LOW / dir_b HIGH (kit wiring is reversed vs the tool's fwd).
    Tune DC_MAX for ground speed; 50 Hz PWM is fine for low-speed creep.
    """
    DEADBAND = 0.05
    DC_MAX = 1400          # gentle (~34% duty @ 50 Hz) — matches the bring-up

    def __init__(self, pca, right, left):
        if right is None or left is None:
            raise NotImplementedError(
                "dc_hbridge mapping incomplete — fill motor_right/left in car_config.")
        self.pca, self.right, self.left = pca, right, left

    def _motor(self, m, linear_x):
        if abs(linear_x) < self.DEADBAND:
            self.pca.set(m["pwm"], 0)                 # coast
            return
        a, b = m["dir"]
        fwd = linear_x > 0
        self.pca.digital(a, not fwd)                  # forward = a LOW, b HIGH
        self.pca.digital(b, fwd)
        self.pca.set(m["pwm"], int(min(1.0, abs(linear_x)) * self.DC_MAX))

    def command(self, linear_x):
        self._motor(self.right, linear_x)
        self._motor(self.left, linear_x)

    def neutral_stop(self):
        self.pca.set(self.right["pwm"], 0)
        self.pca.set(self.left["pwm"], 0)


# ── Node ─────────────────────────────────────────────────────────────────────
class AcdcDriver(Node):
    def __init__(self, name, profile):
        super().__init__('acdc_driver')
        self.p = profile
        bus = profile["i2c_bus"]

        # one PCA9685 instance per unique (bus, addr) — Pro shares 0x40 for both
        self._pcas = {}
        def pca(addr):
            if addr not in self._pcas:
                self._pcas[addr] = Pca9685(bus, addr)
            return self._pcas[addr]

        # steering (universal across platforms)
        self.steer_pca = pca(profile["steer_addr"])
        self.steer_ch = profile["steer_ch"]
        self.center = profile["steer_center"]
        self.s_left = profile["steer_left"]
        self.s_right = profile["steer_right"]
        self.s_lo = min(self.s_left, self.s_right)
        self.s_hi = max(self.s_left, self.s_right)

        # drive backend (platform-specific)
        drive = profile["drive"]
        if drive == "esc":
            self.drive = EscDrive(pca(profile["esc_addr"]), profile["esc_ch"],
                                  profile["esc_neutral"], profile["esc_forward"])
        elif drive == "dc_hbridge":
            try:
                self.drive = DcHbridgeDrive(pca(profile["motor_addr"]),
                                            profile["motor_right"], profile["motor_left"])
            except NotImplementedError as e:
                self.get_logger().warn(f"{e}  ->  throttle DISABLED, steering only.")
                self.drive = NullDrive()
        else:
            raise RuntimeError(f"unknown drive type: {drive!r}")

        if not profile.get("validated", False):
            self.get_logger().warn(
                f"profile '{name}' is NOT validated — running in a safe/limited mode.")

        # center steering + neutral throttle, then go live
        self.steer_pca.set(self.steer_ch, self.center)
        self.drive.neutral_stop()
        self._last_cmd = time.monotonic()
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f"acdc_driver up — platform '{profile['platform']}', drive '{drive}'. "
            f"Listening on /cmd_vel.  *** WHEELS OFF GROUND for first test. ***")

    def _steer(self, z):
        z = max(-1.0, min(1.0, z))
        # +z = LEFT (REP-103). steer_left/right encode any inversion directly.
        if z >= 0:
            pwm = int(self.center + z * (self.s_left - self.center))
        else:
            pwm = int(self.center + (-z) * (self.s_right - self.center))
        self.steer_pca.set(self.steer_ch, max(self.s_lo, min(self.s_hi, pwm)))

    def _on_cmd(self, msg: Twist):
        self._last_cmd = time.monotonic()
        self._steer(msg.angular.z)
        self.drive.command(msg.linear.x)

    def _watchdog(self):
        if time.monotonic() - self._last_cmd > CMD_TIMEOUT_S:
            self.drive.neutral_stop()       # failsafe: stop throttle, hold last steer

    def shutdown(self):
        try:
            self.drive.neutral_stop()
            self.steer_pca.set(self.steer_ch, self.center)
            time.sleep(0.2)
        finally:
            for p in self._pcas.values():
                p.close()


def main():
    name = os.environ.get("ACDC_CAR", car_config.ACTIVE)
    if name not in car_config.PROFILES:
        sys.exit(f"unknown ACDC_CAR '{name}'. options: {list(car_config.PROFILES)}")

    rclpy.init()
    node = AcdcDriver(name, car_config.PROFILES[name])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
