#!/usr/bin/env python3
"""
yellow_fallback.py  —  CON-20260415-08  |  Yellow Phase & Signal Fallback
==========================================================================
4P Code    : 4P-20260415-YELL
Deliverable: YELLOW hold and 2.5-second packet-loss timeout behavior.

CONTRACT PASS when:
  1. Vehicle holds (stops) immediately on YELLOW signal.
  2. Vehicle stops within 2.5 s of last valid UDP packet (packet-loss fallback).
  3. Vehicle resumes automatically when GREEN returns after timeout.

STANDALONE USE:
  python3 yellow_fallback.py   -> runs the 3-behavior self-test with a mock ESC

INTEGRATION:
  This module is used as a standalone contract deliverable.
  In run_main.py, the same three behaviors are enforced directly inside the
  fuse() function and via V2IReceiver.is_live timeout — SignalStateMachine
  is not imported separately. The behaviors are equivalent.

CHANGES FROM ORIGINAL (RALPH fixes):

  FIX 1 — ESC_NEUTRAL and ESC_FORWARD are in MICROSECONDS, not PCA9685 ticks.
    Real hardware uses PCA9685 ticks: NEUTRAL=307, FORWARD=330.
    If you pass set_pwm() (which takes ticks) directly as esc_set_fn, the SSM
    will command 1500 ticks = 7324us, which is outside the ESC's valid range.
    Fix: added us_to_ticks() helper. The standalone test uses _mock_esc which
    just prints the value, so it is unaffected. For real hardware integration,
    wrap set_pwm: esc_set_fn=lambda us: set_pwm(CH_THROTTLE, us_to_ticks(us))

  FIX 2 — _log capped at 50 entries.
    If V2I signal flickers rapidly (noisy WiFi), the state machine fires
    transition events repeatedly. Original _log grew without bound.
    50 entries is enough for contract evidence; transitions are rare in practice.

  FIX 3 — Docstring reference to CON-20260415-07 corrected.
    That contract number does not exist in the repo. V2I UDP is provided by
    v2i_receiver.py (deployed from 3led_v2i.py, which is a frozen base module).

  NO CHANGE — The three behavioral tests are verified correct and unchanged.
"""

import threading
import time

# ── Tunable constants ──────────────────────────────────────────────────────────
PACKET_TIMEOUT_S = 2.5    # seconds without a valid UDP packet -> TIMEOUT state

# NOTE: These constants are in MICROSECONDS to match PIDSteering convention.
# The standalone self-test uses _mock_esc which just prints the value.
# For real hardware: wrap set_pwm with us_to_ticks() — see FIX 1 above.
ESC_NEUTRAL_US = 1499   # us ~ 307 PCA9685 ticks at 50Hz   (motor stopped)
ESC_FORWARD_US = 1611   # us ~ 330 PCA9685 ticks at 50Hz   (demo creep speed)

# Signal string values received over UDP from ESP32 v2i_receiver.py
SIG_RED    = "RED"
SIG_YELLOW = "YELLOW"
SIG_GREEN  = "GREEN"
SIG_NONE   = None


# ── Hardware conversion helper ─────────────────────────────────────────────────

def us_to_ticks(us: float) -> int:
    """
    Convert microseconds to PCA9685 raw ticks (prescale=121, 50Hz).
    Period = 20ms = 20000us across 4096 ticks.

    Used when wrapping set_pwm for real hardware:
        ssm = SignalStateMachine(
            esc_set_fn=lambda us: set_pwm(CH_THROTTLE, us_to_ticks(us))
        )
    """
    return int(round(us * 4096 / 20_000))


# ── Signal state machine ───────────────────────────────────────────────────────

class SignalStateMachine:
    """
    Wraps the V2I signal with YELLOW hold and packet-timeout fallback.

    esc_set_fn must accept a value in the SAME UNIT you choose.
    For the standalone self-test: _mock_esc (prints microseconds).
    For real hardware:
        lambda us: set_pwm(CH_THROTTLE, us_to_ticks(us))

    Call from your control loop:
        ssm.update(received_signal)   # from UDP receive thread
        ssm.tick()                    # every loop tick (aim >= 10 Hz)

    States:
        STOP    -> RED received      -> ESC neutral
        HOLD    -> YELLOW received   -> ESC neutral (same command, distinct state)
        GO      -> GREEN received    -> ESC forward
        TIMEOUT -> no packet 2.5s   -> ESC neutral
    """

    _LOG_MAX = 50   # max state-transition events kept (FIX 2)

    def __init__(self, esc_set_fn):
        self._esc_set = esc_set_fn
        self._last_rx = time.monotonic()
        self._signal  = SIG_NONE
        self._state   = "STOP"
        self._lock    = threading.Lock()
        self._log     = []

    def update(self, signal: str):
        """Call this from your UDP receive thread whenever a packet arrives."""
        with self._lock:
            self._last_rx = time.monotonic()
            self._signal  = signal.upper().strip() if signal else SIG_NONE

    def tick(self) -> int:
        """
        Called every control-loop tick. Evaluates current signal + timeout,
        drives the ESC, and returns the commanded PWM value (in us).
        """
        with self._lock:
            elapsed = time.monotonic() - self._last_rx
            sig     = self._signal

        # Timeout check — highest priority
        if elapsed >= PACKET_TIMEOUT_S:
            if self._state != "TIMEOUT":
                self._state = "TIMEOUT"
                self._log_event("TIMEOUT", f"No packet for {elapsed:.2f}s -> ESC NEUTRAL")
            pwm = ESC_NEUTRAL_US

        elif sig == SIG_RED:
            if self._state != "STOP":
                self._state = "STOP"
                self._log_event("STOP", "RED received -> ESC NEUTRAL")
            pwm = ESC_NEUTRAL_US

        elif sig == SIG_YELLOW:
            if self._state != "HOLD":
                self._state = "HOLD"
                self._log_event("HOLD", "YELLOW received -> vehicle holds")
            pwm = ESC_NEUTRAL_US

        elif sig == SIG_GREEN:
            if self._state != "GO":
                self._state = "GO"
                self._log_event("GO", "GREEN received -> ESC FORWARD")
            pwm = ESC_FORWARD_US

        else:
            # Unknown signal -> safe default
            pwm = ESC_NEUTRAL_US

        self._esc_set(pwm)
        return pwm

    @property
    def state(self) -> str:
        return self._state

    def _log_event(self, state: str, reason: str):
        entry = {
            "timestamp": time.strftime("%H:%M:%S"),
            "state":     state,
            "reason":    reason,
        }
        self._log.append(entry)
        if len(self._log) > self._LOG_MAX:   # FIX 2: cap log
            self._log = self._log[-self._LOG_MAX:]
        print(f"[SSM {entry['timestamp']}] {state}: {reason}")

    def dump_log(self) -> list:
        """Return event log for contract evidence submission."""
        return list(self._log)


# ── Standalone test harness ────────────────────────────────────────────────────

def _mock_esc(pwm_us: int):
    """Prints ESC command instead of writing to hardware. SIMULATION ONLY."""
    print(f"  ESC <- {pwm_us} us  (~{us_to_ticks(pwm_us)} ticks on real hw)")


def run_test():
    """
    Validates the three contract behaviors without any hardware.
    The mock ESC prints what would be sent instead of writing to PCA9685.

    Run: python3 yellow_fallback.py
    """
    print("=== CON-20260415-08 Self-Test  [SIMULATION — no hardware needed] ===\n")
    ssm = SignalStateMachine(_mock_esc)

    def _tick_for(seconds: float, signal: str = None):
        """Tick the SSM for `seconds`, optionally feeding a signal each tick."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if signal:
                ssm.update(signal)
            ssm.tick()
            time.sleep(0.1)

    # Test 1: GREEN then YELLOW — vehicle must hold on YELLOW
    print("\n[TEST 1] GREEN -> YELLOW — vehicle must hold on YELLOW")
    _tick_for(1.0, SIG_GREEN)
    _tick_for(2.0, SIG_YELLOW)
    assert ssm.state == "HOLD", f"Expected HOLD, got {ssm.state}"
    print("  PASS: state == HOLD")

    # Test 2: Packet loss — vehicle must stop within 2.5 s
    print("\n[TEST 2] Signal drops — vehicle must enter TIMEOUT within 2.5 s")
    ssm.update(SIG_GREEN)
    time.sleep(0.05)
    t0 = time.monotonic()
    while ssm.state != "TIMEOUT":
        ssm.tick()
        time.sleep(0.05)
        if time.monotonic() - t0 > 4.0:
            raise AssertionError("TIMEOUT never triggered — check PACKET_TIMEOUT_S")
    elapsed = time.monotonic() - t0
    assert elapsed <= PACKET_TIMEOUT_S + 0.2, \
        f"Took {elapsed:.2f}s — exceeded {PACKET_TIMEOUT_S}s limit"
    print(f"  PASS: TIMEOUT triggered in {elapsed:.2f}s  (limit {PACKET_TIMEOUT_S}s)")

    # Test 3: GREEN returns after timeout — vehicle must resume
    print("\n[TEST 3] GREEN returns after timeout — vehicle must resume (GO)")
    ssm.update(SIG_GREEN)
    ssm.tick()
    time.sleep(0.1)
    ssm.update(SIG_GREEN)
    ssm.tick()
    assert ssm.state == "GO", f"Expected GO, got {ssm.state}"
    print("  PASS: state == GO after GREEN returned")

    print("\n=== All 3 contract behaviors verified ===\n")
    print("Event log (attach as evidence in test_note.md):")
    for e in ssm.dump_log():
        print(f"  [{e['timestamp']}] {e['state']:8s} — {e['reason']}")


if __name__ == "__main__":
    run_test()
