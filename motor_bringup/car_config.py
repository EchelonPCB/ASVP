#!/usr/bin/env python3
"""
car_config.py — per-car calibration. The ACDC fleet is NOT uniform: there are
TWO different kits with different drive hardware AND inverted steering. Set
ACTIVE to the kit you are working on, or import PROFILES[...] in your scripts.
"""

PROFILES = {
    # ── JetRacer PRO AI Kit — the 6 baseline cars (0409 / 0423 code) ──────────
    # Wheels = ESC on PCA9685 CH1 (single servo-style pulse).
    "jetracer_pro": {
        "kit":          "JetRacer Pro AI Kit",
        "i2c_bus":      1,        # bus 0 is DEAD on these
        "pca_addr":     0x40,
        "ch_steer":     0,
        "ch_throttle":  1,
        "steer_center": 320,
        "steer_left":   370,      # higher PWM = LEFT
        "steer_right":  280,      # lower  PWM = RIGHT
        "drive":        "esc",
        "esc_neutral":  307,
        "esc_forward":  330,
        "motor_addr":   0x40,     # same chip, CH1
        "notes":        "ESC on CH1. Proven. bus 0 dead.",
    },

    # ── JetRacer AI Kit (non-Pro) — the new car ───────────────────────────────
    # Wheels = 37-520 DC encoder gearmotors via an H-bridge (NOT an ESC).
    # Needs PWM(speed) + direction pins; driver likely a 2nd PCA9685 ~0x60.
    "jetracer_ai": {
        "kit":          "JetRacer AI Kit (non-Pro)",
        "i2c_bus":      1,        # scan BOTH buses on this car — don't assume bus 0 is dead
        "pca_addr":     0x40,     # steering PCA (VERIFY by scan)
        "ch_steer":     0,
        "steer_center": 425,
        "steer_left":   300,      # lower  PWM = LEFT   (INVERTED vs Pro)
        "steer_right":  500,      # higher PWM = RIGHT
        "drive":        "dc_hbridge",
        "motor_addr":   None,     # UNKNOWN — discover with motor_discovery.py --scan
        "motor_pwm_ch": None,     # to be found
        "motor_dir_ch": None,     # (dirA, dirB) to be found
        "notes":        "DC encoder gearmotors via H-bridge. Browned out near left "
                        "extreme -> separate power rails before pushing. Keep a soft "
                        "left margin if it still dips under load.",
    },
}

ACTIVE = "jetracer_ai"     # <-- set to the car you are bringing up

def active():
    return PROFILES[ACTIVE]
