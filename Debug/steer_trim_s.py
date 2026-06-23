#!/usr/bin/env python3
"""
steer_trim_s.py — Live steering value tester
Type any PWM value to send it immediately
On done or Ctrl+C: saves value to ~/steer_center.txt for running.py
"""

import smbus2
import time
import signal
import sys

BUS       = 1
ADDR      = 0x40
MODE1     = 0x00
PRESCALE  = 0xFE
LED0_ON_L = 0x06

ESC_NEUTRAL = 307

bus = smbus2.SMBus(BUS)

def w(reg, val): bus.write_byte_data(ADDR, reg, val & 0xFF)
def r(reg): return bus.read_byte_data(ADDR, reg)

def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])

last_steer = 320

def safe_stop(sig=None, frame=None):
    print(f"\n[EXIT] Saving STEER_CENTER={last_steer} to steer_center.txt")
    with open('/home/jetson/steer_center.txt', 'w') as f:
        f.write(str(last_steer))
    set_pwm(1, ESC_NEUTRAL)
    # steering held at last value so you can inspect wheel position
    time.sleep(0.5)
    bus.close()
    sys.exit(0)

signal.signal(signal.SIGINT, safe_stop)
signal.signal(signal.SIGTERM, safe_stop)

def init():
    w(MODE1, 0x00); time.sleep(0.1)
    w(MODE1, 0x10); time.sleep(0.1)
    w(PRESCALE, 121); time.sleep(0.1)
    w(MODE1, 0xA1); time.sleep(0.1)
    if r(PRESCALE) != 121:
        raise RuntimeError("PRESCALE wrong. Aborting.")
    if r(MODE1) & 0x10:
        raise RuntimeError("Chip still sleeping. Aborting.")
    print("[init] OK.")

init()
set_pwm(1, ESC_NEUTRAL)
set_pwm(0, 320)
time.sleep(1.0)

print("\nSteer trim — type any PWM value (250–400) and press Enter to apply.")
print("Nudge commands: r (-5 right)  l (+5 left)  rr (-10)  ll (+10)")
print("Type 'done' to save and exit. Ctrl+C also saves.\n")

current = 320

while True:
    set_pwm(0, current)
    last_steer = current
    cmd = input(f"current={current} → ").strip().lower()

    if cmd == 'r':
        current -= 5
    elif cmd == 'l':
        current += 5
    elif cmd == 'rr':
        current -= 10
    elif cmd == 'll':
        current += 10
    elif cmd == 'done':
        last_steer = current
        safe_stop()
    else:
        try:
            val = int(cmd)
            if 250 <= val <= 400:
                current = val
            else:
                print("Value out of range (250–400). Try again.")
        except ValueError:
            print("Unknown command.")
