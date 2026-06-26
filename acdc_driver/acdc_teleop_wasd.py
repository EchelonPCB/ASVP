#!/usr/bin/env python3
"""
acdc_teleop_wasd.py — WASD keyboard teleop -> /cmd_vel (ACDC fleet)
==================================================================
Custom keyboard control for acdc_driver_node.py. Publishes geometry_msgs/Twist
on /cmd_vel; the platform driver maps it to the car, so this works on BOTH Pro
and Reg (steering inversion is handled in the driver, not here).

    w  forward              s  reverse
    a  steer RIGHT          d  steer LEFT          (a/d are intentionally flipped
    z  stop + quit                                  from the usual convention)

Commands LATCH: press once and it holds, republished at 20 Hz so the driver's
0.5 s watchdog stays fed. Press the opposite key to change; press z (or Ctrl-C)
to send a stop and exit the program.

Run in a 2nd container shell (ROS sourced):
    python3 acdc_teleop_wasd.py
*** WHEELS OFF THE GROUND for the first test. ***
"""
import sys
import termios
import tty
import select

import rclpy
from geometry_msgs.msg import Twist

SPEED = 0.5      # linear.x magnitude (gentle; raise toward 1.0 for more speed)
TURN = 1.0       # angular.z magnitude (1.0 = full steering lock)
RATE = 0.05      # publish period in seconds -> 20 Hz

HELP = (
    "\r\n  ACDC WASD teleop -> /cmd_vel\r\n"
    "    w forward     s reverse\r\n"
    "    a steer right     d steer left\r\n"
    "    z stop + quit\r\n"
    "  commands latch (held until changed). WHEELS OFF GROUND for first test.\r\n\r\n"
)


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('acdc_teleop_wasd')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    lin = 0.0
    ang = 0.0
    sys.stdout.write(HELP)
    sys.stdout.flush()
    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            r, _, _ = select.select([sys.stdin], [], [], RATE)
            if r:
                k = sys.stdin.read(1).lower()
                if k == 'w':
                    lin = SPEED
                elif k == 's':
                    lin = -SPEED
                elif k == 'a':
                    ang = -TURN          # steer RIGHT (negative = toward steer_right)
                elif k == 'd':
                    ang = TURN           # steer LEFT  (positive = toward steer_left)
                elif k == 'z' or k == '\x03':   # z or Ctrl-C -> stop + quit
                    break
            t = Twist()
            t.linear.x = lin
            t.angular.z = ang
            pub.publish(t)
            drive = 'fwd' if lin > 0 else 'rev' if lin < 0 else 'idle'
            turn = 'right' if ang < 0 else 'left' if ang > 0 else 'center'
            sys.stdout.write(f"\r  throttle {lin:+.2f}  steer {ang:+.2f}   ({drive}, {turn})    ")
            sys.stdout.flush()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        pub.publish(Twist())         # final zero so the car stops
        node.destroy_node()
        rclpy.shutdown()
        sys.stdout.write("\r\n  stopped — teleop exited.\r\n")
        sys.stdout.flush()


if __name__ == '__main__':
    main()
