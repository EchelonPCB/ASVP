#!/usr/bin/env python3
"""
T2 · 02_pwm_test.py — ACDC JetRacer Pro · in-container PWM hardware test
========================================================================
Proves the Docker container can actually DRIVE the PCA9685 (steering + ESC)
over I2C bus 1 — the real "does Docker break hardware access" test.

Uses the AUTHORITATIVE ACDC hardware constants (procedure doc / demoday.py).

SAFE BY DEFAULT — sweeps STEERING only. The ESC/throttle test is gated behind
--throttle and the WHEELS MUST BE OFF THE GROUND.

Run inside the ROS 2 container (needs smbus2 — `pip3 install smbus2` if missing):
    python3 02_pwm_test.py              # steering sweep only (safe)
    python3 02_pwm_test.py --throttle   # adds a brief ESC test — WHEELS OFF GROUND
"""
import sys
import time
import signal

try:
    import smbus2
except ImportError:
    sys.exit("smbus2 not found. Inside the container run:  pip3 install smbus2")

# ── Authoritative ACDC hardware constants (do not change) ──────────────────
BUS = 1
ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
CH_STEER = 0
CH_THROTTLE = 1
ESC_NEUTRAL = 307          # motor stopped — always return here
ESC_FORWARD = 325          # gentle creep (procedure-doc demo speed)
STEER_CENTER = 320
STEER_RIGHT_LIMIT = 280    # soft stop
STEER_LEFT_LIMIT = 370     # soft stop

bus = smbus2.SMBus(BUS)


def w(reg, val):
    bus.write_byte_data(ADDR, reg, val & 0xFF)


def r(reg):
    return bus.read_byte_data(ADDR, reg)


def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])


def init_pca9685():
    w(MODE1, 0x00); time.sleep(0.1)
    w(MODE1, 0x10); time.sleep(0.1)      # sleep to allow PRESCALE write
    w(PRESCALE, 121); time.sleep(0.1)    # 121 -> 50 Hz
    w(MODE1, 0xA1); time.sleep(0.1)      # wake + auto-increment + allcall
    ps = r(PRESCALE)
    m = r(MODE1)
    print(f"[init] MODE1={hex(m)}  PRESCALE={ps}")
    if ps != 121:
        raise RuntimeError(f"PRESCALE readback {ps} != 121 — I2C wiring/bus problem")
    if m & 0x10:
        raise RuntimeError("Chip still in sleep mode")
    print("[init] PCA9685 verified OK (bus 1, 0x40, 50 Hz)")


def center():
    set_pwm(CH_THROTTLE, ESC_NEUTRAL)
    set_pwm(CH_STEER, STEER_CENTER)


def safe_stop(*_a):
    print("\n[safe] centering steering + ESC neutral")
    try:
        center()
        time.sleep(0.3)
    finally:
        try:
            bus.close()
        except Exception:
            pass
    sys.exit(0)


signal.signal(signal.SIGINT, safe_stop)
signal.signal(signal.SIGTERM, safe_stop)


def _ramp(ch, start, end, step, dwell=0.02):
    rng = range(start, end + (1 if step > 0 else -1), step)
    for t in rng:
        set_pwm(ch, t)
        time.sleep(dwell)


def sweep_steering():
    print("[steer] center");        set_pwm(CH_STEER, STEER_CENTER); time.sleep(1.0)
    print("[steer] -> RIGHT limit");_ramp(CH_STEER, STEER_CENTER, STEER_RIGHT_LIMIT, -2); time.sleep(0.5)
    print("[steer] -> center");     _ramp(CH_STEER, STEER_RIGHT_LIMIT, STEER_CENTER, +2); time.sleep(0.5)
    print("[steer] -> LEFT limit"); _ramp(CH_STEER, STEER_CENTER, STEER_LEFT_LIMIT, +2);  time.sleep(0.5)
    print("[steer] -> center");     _ramp(CH_STEER, STEER_LEFT_LIMIT, STEER_CENTER, -2)
    print("[steer] PASS if the wheels turned right, then left, then re-centered.")


def throttle_test():
    print("\n*** ESC TEST — WHEELS MUST BE OFF THE GROUND ***")
    for n in range(5, 0, -1):
        print(f"   starting in {n}...  (Ctrl+C to abort)")
        time.sleep(1)
    print("[esc] arming (neutral 2 s)"); set_pwm(CH_THROTTLE, ESC_NEUTRAL); time.sleep(2.0)
    print(f"[esc] FORWARD {ESC_FORWARD} for 1.5 s"); set_pwm(CH_THROTTLE, ESC_FORWARD); time.sleep(1.5)
    print("[esc] neutral"); set_pwm(CH_THROTTLE, ESC_NEUTRAL); time.sleep(0.5)
    print("[esc] PASS if the wheels spun forward briefly then stopped.")
    print("      (No spin? ESC may need its power-on arming — power-cycle the ESC")
    print("       with throttle at neutral, then re-run.)")


if __name__ == "__main__":
    print("=== T2 PWM hardware test ===")
    init_pca9685()
    center()
    time.sleep(1.0)
    sweep_steering()
    if "--throttle" in sys.argv:
        throttle_test()
    else:
        print("\n[info] Steering only. Add --throttle (wheels OFF ground) to test the ESC.")
    safe_stop()
