#!/usr/bin/env python3
"""
car_config.py — ACDC fleet PLATFORM profiles (the single source of truth).
==============================================================================
TWO JetRacer platforms, NOT interchangeable. The Docker image and the driver
node are IDENTICAL fleet-wide; the ONLY thing that changes per platform is the
profile below.

  jetracer_pro  — the 6 baseline cars (0409 / 0423 / demoday).
                  Drive = ESC pulse on PCA9685 @ 0x40 CH1.  ✅ VALIDATED.

  jetracer_reg  — the regular (NON-Pro) JetRacer.
                  Steering = PCA9685 @ 0x40 (inverted vs Pro).
                  Drive    = DC gearmotors via H-bridge on a 2nd PCA9685 @ 0x60.
                  ⚠️ ON HOLD: right motor confirmed, LEFT motor not yet
                  bench-validated. validated=False keeps the driver from
                  pushing throttle on this platform until you sign it off.

Select the platform of the car you're flashing with the env var ACDC_CAR, or by
setting ACTIVE below:
    ACDC_CAR=jetracer_pro python3 acdc_driver_node.py
"""

PROFILES = {
    # ── JetRacer PRO — 6 cars, fully validated (0409 / 0423 / demoday) ─────────
    "jetracer_pro": {
        "platform":     "JetRacer Pro",
        "validated":    True,
        "i2c_bus":      1,            # bus 0 is dead on these
        # steering (PCA9685)
        "steer_addr":   0x40,
        "steer_ch":     0,
        "steer_center": 320,
        "steer_left":   370,          # higher PWM = LEFT
        "steer_right":  280,          # lower  PWM = RIGHT
        # drive — ESC on the SAME chip, CH1
        "drive":        "esc",
        "esc_addr":     0x40,
        "esc_ch":       1,
        "esc_neutral":  307,          # motor stopped — always return here
        "esc_forward":  325,          # gentle creep (proven; 325-330 usable)
        "notes":        "Proven on 0409/0423/demoday. Single PCA9685 @ 0x40: "
                        "CH0 steer, CH1 ESC. bus 0 dead.",
    },

    # ── JetRacer REG (non-Pro) — today's car. ON HOLD until left motor found ──
    "jetracer_reg": {
        "platform":     "JetRacer (regular / non-Pro)",
        "validated":    True,         # both motors bench-confirmed
        "i2c_bus":      1,
        # steering (PCA9685) — CONFIRMED @ 0x40, but INVERTED vs Pro
        "steer_addr":   0x40,
        "steer_ch":     0,
        "steer_center": 425,
        "steer_left":   300,          # lower PWM = LEFT (inverted vs Pro)
        "steer_right":  500,          # higher PWM = RIGHT
        # drive — 2 DC gearmotors via TB6612 H-bridge on a 2nd PCA9685 @ 0x60.
        # NOTE: this board does NOT use the standard Waveshare Motor-B map — the
        # channels below are the BENCH-CONFIRMED ones. Both motors read
        # "backwards then forwards" under the discovery tool, i.e. the kit wiring
        # is polarity-reversed vs the tool's fwd; the driver flips it so
        # linear.x > 0 = real forward.
        "drive":        "dc_hbridge",
        "motor_addr":   0x60,         # CONFIRMED
        "motor_right":  {"pwm": 0, "dir": (1, 2)},   # CONFIRMED (Motor A)
        "motor_left":   {"pwm": 6, "dir": (7, 8)},   # CONFIRMED (Motor B — NOT 5/3,4)
        "notes":        "Both drive motors confirmed on 0x60: right pwm0 dir1,2, "
                        "left pwm6 dir7,8. Other bus-1 devices: 0x41 INA219 "
                        "(battery), 0x70 PCA all-call, 0x3c OLED.",
    },
}

ACTIVE = "jetracer_pro"   # <-- the platform you are flashing (or set env ACDC_CAR)


def active():
    return PROFILES[ACTIVE]


def get(name):
    return PROFILES[name]
