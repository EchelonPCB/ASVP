#!/usr/bin/env python3
"""
pid_steer.py  —  CON-20260415-10  |  PID Steering Controller
=============================================================
4P Code    : 4P-20260415-PID
Deliverable: PID function that takes lane offset (from CON-09) and outputs
             servo PWM correction for the PCA9685 steering channel.

PASS threshold: Vehicle completes one full lap staying between white lines
                with NO manual steering input applied.

CHANGES FROM ORIGINAL (RALPH fixes):

  FIX 1 — _log list capped at 200 entries inside compute().
    Original: _log grew by 1 dict every tick. At 30fps for 10 min = 18,000
    dict entries held in RAM on a 4GB Jetson Nano.
    Fix: after appending, trim to last 200 entries. save_tuning_notes() still
    gets the 20 most recent entries it needs. dump_tuning_notes() is unchanged.

  FIX 2 — PWM_MIN and PWM_MAX corrected to match hardware.
    Original: PWM_MIN=1200us, PWM_MAX=1800us (generic defaults).
    Real hardware soft limits from procedure doc:
      STEER_RIGHT_LIMIT = 280 ticks = 1367us
      STEER_LEFT_LIMIT  = 370 ticks = 1807us
    With 1200us the PID could physically force the servo past its stop.
    Fix: PWM_MIN=1367, PWM_MAX=1807. If caller needs wider range, override
    as instance attributes after construction (pid.PWM_MIN = X).

  FIX 3 — load_steer_center() removed. steer_center.txt does not exist.
    Original function read a file and fell back to 1500us if not found.
    The procedure doc and run_main.py both confirm STEER_CENTER is hardcoded
    as 320 ticks = 1563us. The file was a legacy artifact. 1500us is wrong.
    Fix: removed load_steer_center(). STEER_CENTER_US constant added instead.

  FIX 4 — import math removed. It was imported but never used.

  FIX 5 — Simulation updated to use correct hardware center (1563us).
    Original simulation used steer_center=1500 which is not our hardware value.

Deploy to:
  /home/jetson/jetson/ssunc_perception/pid_steer.py

Integration in run_main.py:
    from pid_steer import PIDSteering, STEER_CENTER_US
    pid = PIDSteering(steer_center=STEER_CENTER_US)
    # inside GREEN control loop:
    steer_us    = pid.compute(lane_offset)   # lane_offset in pixels from lane_detect
    steer_ticks = us_to_ticks(steer_us)      # convert to PCA9685 ticks
    set_pwm(CH_STEER, steer_ticks)
"""

import time

# ── Hardware center ─────────────────────────────────────────────────────────────
# STEER_CENTER = 320 PCA9685 ticks = 1562.5us
# Derived from: ticks * 20000us / 4096 at prescale=121 (50Hz)
# Source: ACDC JetRacer Pro hardware constants table (procedure doc, Section 7)
STEER_CENTER_US = 1563   # us — use this when constructing PIDSteering


# ── PID steering controller ────────────────────────────────────────────────────

class PIDSteering:
    """
    PID controller that converts lane offset (pixels) to servo PWM in microseconds.

    The output is in MICROSECONDS. run_main.py converts to PCA9685 ticks with:
        ticks = round(us * 4096 / 20000)

    Offset sign convention (from lane_detect.py):
        negative -> vehicle too far LEFT  -> steer RIGHT -> output < steer_center
        positive -> vehicle too far RIGHT -> steer LEFT  -> output > steer_center

    Servo PWM convention (verify on your hardware — servo wiring may invert):
        steer_center     -> straight ahead
        steer_center + N -> turns LEFT
        steer_center - N -> turns RIGHT

    Starting gains (tune on the real track — see TUNING_GUIDE below):
        Kp = 0.8   proportional: immediate response to offset
        Ki = 0.01  integral: corrects steady-state drift
        Kd = 0.3   derivative: damps oscillation on curves
    """

    # Default gains — MUST be validated and tuned on real track
    DEFAULT_KP = 0.8
    DEFAULT_KI = 0.01
    DEFAULT_KD = 0.3

    # Servo PWM limits in microseconds — matched to hardware soft stops (FIX 2)
    # 280 ticks = 1367us (right limit)    370 ticks = 1807us (left limit)
    PWM_MIN = 1367   # us — right soft stop
    PWM_MAX = 1807   # us — left soft stop

    # Max integral term (anti-windup)
    INTEGRAL_LIMIT = 200.0

    # Max log entries kept in memory (FIX 1)
    _LOG_MAX = 200

    def __init__(self,
                 steer_center: int = STEER_CENTER_US,
                 kp: float = DEFAULT_KP,
                 ki: float = DEFAULT_KI,
                 kd: float = DEFAULT_KD):
        self.steer_center = steer_center
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()
        self._log        = []   # capped at _LOG_MAX entries (FIX 1)

    def compute(self, offset: float | None) -> int:
        """
        Compute servo PWM for this control tick.

        Args:
            offset : pixel offset from lane_detect.get_offset()
                     None -> detection lost, return straight (steer_center)

        Returns:
            int PWM in microseconds, clamped to [PWM_MIN, PWM_MAX]
        """
        if offset is None:
            # Detection lost — hold straight. Do NOT update integrator.
            return self.steer_center

        now  = time.monotonic()
        dt   = now - self._prev_time
        if dt <= 0:
            dt = 0.01
        self._prev_time = now

        error = offset

        # Proportional
        p = self.kp * error

        # Integral (with anti-windup clamp)
        self._integral += error * dt
        self._integral  = max(-self.INTEGRAL_LIMIT,
                              min(self.INTEGRAL_LIMIT, self._integral))
        i = self.ki * self._integral

        # Derivative
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        correction = p + i + d

        pwm = int(self.steer_center + correction)
        pwm = max(self.PWM_MIN, min(self.PWM_MAX, pwm))

        # Log tick for tuning evidence — trim to avoid unbounded growth (FIX 1)
        self._log.append({
            "t":      round(now, 3),
            "offset": round(offset, 2),
            "p":      round(p, 2),
            "i":      round(i, 2),
            "d":      round(d, 2),
            "pwm":    pwm,
        })
        if len(self._log) > self._LOG_MAX:
            self._log = self._log[-self._LOG_MAX:]

        return pwm

    def reset(self):
        """
        Reset integrator and derivative state.
        Call this whenever the car stops (RED/YELLOW/FAILSAFE) so accumulated
        error from sitting at a red light doesn't carry over into the next GO.
        """
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()

    def dump_tuning_notes(self) -> list:
        """Return a copy of the current log for evidence submission."""
        return list(self._log)

    def save_tuning_notes(self, path: str = "tuning_notes.txt"):
        """Write Kp/Ki/Kd and the 20 most recent log entries to a text file."""
        with open(path, "w") as f:
            f.write("CON-20260415-10 — PID Tuning Notes\n")
            f.write("=" * 40 + "\n\n")
            f.write(f"STEER_CENTER : {self.steer_center} us\n")
            f.write(f"Kp           : {self.kp}\n")
            f.write(f"Ki           : {self.ki}\n")
            f.write(f"Kd           : {self.kd}\n\n")
            f.write(f"PWM clamp    : [{self.PWM_MIN}, {self.PWM_MAX}] us\n\n")
            f.write("Last 20 ticks:\n")
            f.write(f"{'t':>8}  {'offset':>8}  {'P':>7}  {'I':>7}  {'D':>7}  {'PWM':>6}\n")
            for entry in self._log[-20:]:
                f.write(
                    f"{entry['t']:>8.3f}  {entry['offset']:>8.2f}  "
                    f"{entry['p']:>7.2f}  {entry['i']:>7.2f}  "
                    f"{entry['d']:>7.2f}  {entry['pwm']:>6d}\n"
                )
        print(f"[PID] Tuning notes saved -> {path}")


# ── Tuning guide ───────────────────────────────────────────────────────────────

TUNING_GUIDE = """
PID TUNING PROCEDURE — CON-20260415-10
=======================================
Run this sequence on the real ASVP track in CKB 110.
Document every change in tuning_notes.txt for evidence submission.

STEP 1 — Set Ki=0, Kd=0. Increase Kp until vehicle tracks lane but oscillates.
STEP 2 — Back off Kp by 20%. Add Kd gradually to damp oscillation.
STEP 3 — Add small Ki (start 0.005) only if vehicle consistently drifts one side.
STEP 4 — Run one full lap. If PASS -> record final values and save tuning notes.

Starting point (ASVP RC vehicle):
    Kp=0.8  Ki=0.01  Kd=0.3

Hardware center: 1563us (320 ticks at 50Hz PCA9685)
Clamp range:     1367us - 1807us (280 - 370 ticks)

Common problems:
    Oscillates wildly  -> reduce Kp or increase Kd
    Slow to correct    -> increase Kp
    Drifts one side    -> increase Ki slightly
    Overshoots curves  -> increase Kd
"""


# ── Standalone simulation (no camera, no hardware) ────────────────────────────

def run_simulation():
    """
    Simulate a vehicle drifting off-center and verify PID pulls it back.
    Uses the correct hardware center (1563us). No camera or hardware needed.

    Run: python3 pid_steer.py
    """
    print("=== CON-20260415-10 PID Simulation ===\n")
    print(TUNING_GUIDE)

    pid = PIDSteering(steer_center=STEER_CENTER_US)   # 1563us (FIX 5)

    # Simulated offsets: starts centered, drifts right, PID corrects back
    offsets = [0, 5, 15, 30, 45, 35, 20, 8, 2, 0, -5, -10, -5, 0]

    print(f"\n{'Tick':>4}  {'Offset':>8}  {'PWM':>6}  Direction")
    print("-" * 42)
    for tick, off in enumerate(offsets):
        pwm = pid.compute(float(off))
        if pwm > STEER_CENTER_US + 20:
            direction = "<- LEFT  (steer left)"
        elif pwm < STEER_CENTER_US - 20:
            direction = "-> RIGHT (steer right)"
        else:
            direction = "STRAIGHT"
        print(f"{tick:>4}  {off:>8.1f}  {pwm:>6}  {direction}")
        time.sleep(0.1)

    pid.save_tuning_notes("tuning_notes_sim.txt")
    print("\nSimulation complete. See tuning_notes_sim.txt")


if __name__ == "__main__":
    run_simulation()
