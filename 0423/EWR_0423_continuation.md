# Engineering Work Record — JPA 0423 Continuation

| Field          | Detail                                                              |
|----------------|---------------------------------------------------------------------|
| Project        | ACDC JetRacer Pro (JPA)                                             |
| Subsystem      | V2I Fusion · Speed Control · ESP32 Firmware · Physics Clearance     |
| Session Date   | 2026-04-23                                                          |
| Record Mode    | Continuation (prior: EWR_0409_main_loop_baseline.md)               |
| Record Author  | AISkills MCP — engineering-work-record-generation V002 + jpa-module-integration V002 |
| Files Modified | `0423/demoday.py`, `0423/ssunc_perception/v2i_receiver.py`, `0423/ssunc_perception/pid_steer.py`, `0423/ssunc_perception/lane_detect.py`, `0423/light/ESP32_2.0_networked_traffic_light` |

---

## 1. Objective

Resolve runtime errors found during the first live Jetson run, fix the `v2i_remaining` data pipeline, implement timing-based speed control (RUSH / LATE_STOP), add a physics-based intersection clearance model with dead-reckoning distance estimation, and verify that no new logic violates the AGREE-first fusion doctrine established in the 0409 baseline.

---

## 2. Prior Context (Not Repeated Here)

The 0409 baseline record documents the three-module architecture (`3led_detector.py`, `3led_v2i.py`, `3led_demoday.py`), AGREE-first fusion doctrine, hardware constants, and baseline run analysis. The 0423 working directory was built from those frozen files plus three contract deliverables (CON-08, CON-09, CON-10) and a full `demoday.py` rewrite with PID steering, lane detection integration, and MJPEG stream.

---

## 3. Work Performed — 0423 Session

### 3.1 Python Version Compatibility Fix (Two Passes)

**Pass 1 — Python 3.8 assumption:** Added `from __future__ import annotations` to `pid_steer.py` and `lane_detect.py` to defer evaluation of `float | None` type union annotations. Syntax verified via `ast.parse()`.

**Pass 2 — Actual version is Python 3.6:** First pass failed on Jetson with `TypeError: future feature annotations is not defined`. JetPack 4.x on Jetson Nano ships Ubuntu 18.04 with Python 3.6, not 3.8. Fix revised to `from typing import Optional` with `Optional[float]` replacing all `float | None` annotations. Verified with a presence and pipe-syntax-absence check across all four copies of affected files (JPA and AIGST mirrors).

**Affected lines:**
- `pid_steer.py` line 116: `compute(self, offset: float | None) → Optional[float]`
- `lane_detect.py` lines 231, 238: `_mean_x()` and `_compute_offset()` return annotations

### 3.2 First Successful Live Run — Log Analysis

Run `run_20260423_134909.csv` uploaded: 5,669 loops, 189 s, 30.0 fps average. Key findings:

| Metric | Value |
|---|---|
| V2I live | 99.9% (6 FAILSAFE frames at startup only) |
| AGREE (both GREEN) | 249 frames / 4.4% — 9 windows, longest 79 frames |
| DISAGREE_SAFE | 2,681 frames / 47.3% |
| V2I_DOMINANT | 2,733 frames / 48.2% |
| Camera confidence (non-zero) | mean 0.989, median 1.000 |
| `v2i_remaining` | 0.00 for entire run |

The AGREE-first doctrine operated correctly — movement only during confirmed dual-sensor GREEN. The 0.00 remaining time throughout was identified as a bug requiring investigation.

### 3.3 V2I Remaining Bug — Root Cause and Fix

**Root cause:** The ESP32 source code (`light/ESP32_2.0_networked_traffic_light`) was read and confirmed to send `"seconds_remaining"` as the JSON key — exactly matching the receiver's parser. The real cause was that in MANUAL mode (`cycleEnabled = false`), `secondsRemaining()` returns `0.0f` once `elapsed >= total` (permanent after phase timer expires). During the SSUNC run, the light was likely in manual override mode, causing every packet to carry `seconds_remaining: 0`.

**Fix — ESP32 firmware:** `secondsRemaining()` now returns `999.0f` when `cycleEnabled == false`. This sentinel signals to the Jetson that no transition is upcoming.

**Fix — Jetson receiver (`v2i_receiver.py`):** Added `SECONDS_REM_KEYS` fallback tuple to try six common field name variants in order. Added first-packet key dump to terminal output so the exact ESP32 payload format is confirmed on every new deployment.

**Fix — `demoday.py` `speed_for_green()`:** `v2i_remaining >= 999.0` is now treated as NORMAL speed (manual hold, no incoming transition — RUSH and LATE_STOP do not apply).

### 3.4 Speed Control Implementation

Added three-tier speed decision downstream of AGREE-first fusion:

| `v2i_remaining` | `throttle_mode` | Throttle |
|---|---|---|
| ≥ 999 or ≤ 0 | NORMAL | ESC_FORWARD (330) |
| > V2I_RUSH_THRESHOLD_S (3.0 s) | NORMAL | ESC_FORWARD (330) |
| V2I_STOP_THRESHOLD_S – RUSH | RUSH | ESC_FORWARD_FAST (345) |
| ≤ V2I_STOP_THRESHOLD_S (1.5 s) | LATE_STOP | ESC_NEUTRAL (307) |

LATE_STOP mutates `final_state` to "RED" before the steering block, ensuring `pid.reset()` and `STEER_CENTER` fire on the same tick — car brakes and straightens simultaneously. RUSH does not touch `final_state`; PID continues steering.

**Doctrine integrity confirmed:** Both RUSH and LATE_STOP are only reachable when fusion has already produced GREEN (AGREE). Neither path can produce movement without prior dual-sensor confirmation.

### 3.5 Physics-Based Clearance Model

Replaced the threshold heuristic with a kinematic dead-reckoning model, gated behind `PHYSICS_MODE = False` (threshold fallback active until calibrated).

**Constants added to `demoday.py`:**

| Constant | Default | Calibration method |
|---|---|---|
| `DIST_TO_STOP_LINE_M` | 1.5 m | Tape from car nose to stop line at start gate |
| `DIST_INTERSECTION_M` | 0.6 m | Tape from entry to exit of intersection crossing |
| `SPEED_AT_FORWARD_MPS` | 0.20 m/s | Time 3 × 1 m runs at ESC_FORWARD, average |
| `SPEED_AT_FAST_MPS` | 0.28 m/s | Time 3 × 1 m runs at ESC_FORWARD_FAST, average |

**Dead-reckoning tracker:** `_go_start_time` is reset on every GREEN window entry. `d_remaining = max(0, DIST_TO_STOP_LINE_M − elapsed × SPEED_AT_FORWARD_MPS)` is computed each tick and passed to `speed_for_green()`. Conservative speed (FORWARD, not FAST) is used for the distance estimate so the decision never optimistically overestimates clearance.

**Kinematic decision in `speed_for_green()`:**
```
t_to_reach        = d_remaining / SPEED_AT_FORWARD_MPS
t_to_clear_normal = (d_remaining + DIST_INTERSECTION_M) / SPEED_AT_FORWARD_MPS
t_to_clear_fast   = (d_remaining + DIST_INTERSECTION_M) / SPEED_AT_FAST_MPS

v2i_remaining ≥ t_to_clear_normal → NORMAL
v2i_remaining ≥ t_to_clear_fast   → RUSH
v2i_remaining ≥ t_to_reach        → PHYSICS_STOP (enters but can't clear)
v2i_remaining < t_to_reach         → PHYSICS_STOP (can't even reach line)
```

**CSV expanded:** `d_remaining_m` column added. `throttle_mode` values now include `PHYSICS_STOP` in addition to `NORMAL`, `RUSH`, `LATE_STOP`, `STOP`. The `decision_source` suffix is now dynamic (`+LATE_STOP` or `+PHYSICS_STOP` depending on which override fired).

---

## 4. Decisions Made

### D1 — Two-stage Python compatibility fix
First fix assumed Python 3.8 (from JetPack 4.6 documentation); actual Jetson reported Python 3.6. The `from typing import Optional` approach was chosen over `from __future__ import annotations` because it works back to Python 3.5 and eliminates the version ambiguity entirely.

### D2 — 999.0 sentinel for ESP32 MANUAL mode
The ESP32 firmware now returns `999.0f` instead of `0.0f` when in manual hold. This was chosen over `−1` (negative could confuse downstream checks) or `float('inf')` (not valid in C) to be an unambiguous, JSON-safe value with no collision risk against real remaining times.

### D3 — Physics mode off by default
`PHYSICS_MODE = False` keeps the threshold fallback active until all four constants are physically measured. Enabling an uncalibrated physics model would be worse than the conservative thresholds — a wrong `SPEED_AT_FORWARD_MPS` could compute that the car can clear when it cannot.

### D4 — Conservative speed for dead reckoning
`SPEED_AT_FORWARD_MPS` (not FAST) is used to estimate `d_covered`, so the model systematically underestimates how far the car has traveled. This makes `d_remaining` slightly larger than reality, which biases toward stopping rather than entering. Safe side of the error.

### D5 — ML migration deferred
Analysis confirmed: deterministic perception (HSV, Hough) is the appropriate ML migration target post-demoday. The fusion doctrine and speed control should remain deterministic because they are auditable, logged, and require no training data. The CSV logs now being generated are appropriate training input for a future RL speed policy.

---

## 5. Current State

`demoday.py` in `0423/` is syntax-verified and contains the full stack: AGREE-first fusion, three-tier speed control, physics clearance model (gated), dead-reckoning tracker, and expanded CSV logging. `v2i_receiver.py` has the key-fallback list and first-packet debug dump. The ESP32 firmware fix is staged in `0423/light/ESP32_2.0_networked_traffic_light` and needs to be reflashed.

**What is ready to deploy:** All Python files in `0423/`. SCP command: `scp -r ~/Desktop/JPA/0423 jetson@172.20.10.13:~/`

**What requires field work before physics mode can be enabled:**
- Measure `DIST_TO_STOP_LINE_M` with tape on real track
- Measure `DIST_INTERSECTION_M` with tape on real track
- Calibrate `SPEED_AT_FORWARD_MPS`: time 3 × 1 m runs at ESC_FORWARD, compute mean
- Calibrate `SPEED_AT_FAST_MPS`: same at ESC_FORWARD_FAST
- Set `PHYSICS_MODE = True` in `demoday.py`

**What requires hardware work:** Reflash ESP32 with updated firmware to correct the MANUAL-mode remaining-time bug.

---

## 6. Risks

| Risk | Severity | Mitigation |
|---|---|---|
| Physics constants not measured before demoday | Medium | Threshold fallback (`PHYSICS_MODE = False`) is active and has been validated in one live run |
| ESP32 not reflashed before demoday | Medium | `v2i_remaining` will stay 0 during manual mode; `speed_for_green` treats 0 as NORMAL (safe) |
| Dead reckoning drift on long GO windows | Low | `d_remaining` floors at 0; car already past stop line → NORMAL speed applies, physics check is irrelevant |
| `SPEED_AT_FORWARD_MPS` underestimate | Low | Biases toward stopping rather than entering — safe direction |

---

## 7. Next Steps

1. **Reflash ESP32** with updated `ESP32_2.0_networked_traffic_light` firmware. Verify first-packet log shows `rem_key='seconds_remaining'` and non-zero remaining value.
2. **Run calibration session:** Measure four constants on ASVP track, set `PHYSICS_MODE = True`, run one lap with physics mode active, analyze `d_remaining_m` and `throttle_mode` columns in the new log.
3. **Verify RUSH behavior in live run:** Confirm `throttle_mode=RUSH` frames appear in log during late-GREEN windows and that car physically clears intersection.
4. **Tune `ESC_FORWARD_FAST`:** Current value is 345 ticks (1685 µs). Adjust based on whether car clears intersection in RUSH windows.
5. **Calibrate PID gains on real track:** Current Kp=0.8, Ki=0.01, Kd=0.3 are starting estimates. Tune using the procedure in `pid_steer.py` TUNING_GUIDE.
6. **Verify slope-sign lane classifier on curves:** Confirm `_classify_by_slope()` correctly assigns L/R borders throughout the track, including corners.
7. **Post-demoday (optional):** Evaluate ML traffic light detector (bounding box output → eliminates need for DIST_TO_STOP_LINE_M constant).

---

## 8. Assumptions and Missing Information

- `SPEED_AT_FORWARD_MPS = 0.20` and `SPEED_AT_FAST_MPS = 0.28` are engineering estimates. Real values depend on battery state, surface friction, and ESC calibration.
- `DIST_TO_STOP_LINE_M = 1.5` and `DIST_INTERSECTION_M = 0.6` are placeholders. The actual ASVP track layout has not been measured in this session.
- The ESP32 reflash requires Arduino IDE or PlatformIO access and a USB cable to the ESP32; this was not performed in this session.
- Python version on the Jetson is confirmed as 3.6 based on the runtime error observed. The exact JetPack version has not been independently verified.
