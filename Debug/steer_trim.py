#!/usr/bin/env python3
"""
steer_trim.py — Find true steering center under load
Ctrl+C at any time → safe exit
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

ESC_NEUTRAL  = 307
STEER_CENTER = 320  # starting guess

bus = smbus2.SMBus(BUS)

def w(reg, val): bus.write_byte_data(ADDR, reg, val & 0xFF)
def r(reg): return bus.read_byte_data(ADDR, reg)

def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])

def safe_stop(sig=None, frame=None):
    print("\n[STOP] Centering and exiting.")
    set_pwm(1, ESC_NEUTRAL)
    set_pwm(0, STEER_CENTER)
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
    mode1_val    = r(MODE1)
    prescale_val = r(PRESCALE)
    if prescale_val != 121:
        raise RuntimeError("PRESCALE wrong. Aborting.")
    if mode1_val & 0x10:
        raise RuntimeError("Chip still sleeping. Aborting.")
    print("[init] OK.")

init()
set_pwm(1, ESC_NEUTRAL)
set_pwm(0, STEER_CENTER)
time.sleep(1.0)

current = STEER_CENTER
print(f"\nCurrent steering value: {current}")
print("Commands: 'r' = nudge right (+5), 'l' = nudge left (-5), 'rr' = big right (+10), 'll' = big left (-10)")
print("          'd' = done (prints final center value), 'q' = quit\n")

while True:
    set_pwm(0, current)
    cmd = input(f"val={current} → command: ").strip().lower()

    if cmd == 'r':
        current -= 5   # lower value = right
    elif cmd == 'l':
        current += 5   # higher value = left
    elif cmd == 'rr':
        current -= 10
    elif cmd == 'll':
        current += 10
    elif cmd == 'd':
        print(f"\n>>> TRUE STEER_CENTER = {current}")
        print(f"Update run.py: STEER_CENTER = {current}")
        break
    elif cmd == 'q':
        break
    else:
        print("Unknown command.")

safe_stop()
