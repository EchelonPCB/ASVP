# Engineering Work Record — JPA 0409 Main Loop (Baseline)

| Field            | Detail                                                        |
|------------------|---------------------------------------------------------------|
| Project          | ACDC JetRacer Pro (JPA)                                       |
| Subsystem        | Traffic Light Perception + V2I Fusion + Drive Control         |
| Session Date     | 2026-04-09 (code authored) / Reviewed 2026-04-22             |
| Record Mode      | Baseline                                                      |
| Record Author    | AISkills MCP — engineering-work-record-generation V002        |
| Files Reviewed   | `0409/Dev/3led_detector.py`, `3led_v2i.py`, `3led_demoday.py` |

---

## 1. Objective

Document the architecture, behavior, and engineering status of the three-file main loop system produced for ACDC Demoday (SSUNC Phase 2). Identify strengths, risks, and next steps with supporting evidence from the source code.

---

## 2. Background Context

The JPA platform is a Jetson Nano–based JetRacer Pro RC car operated within the SSUNC
multi-institution autonomous systems showcase. The 0409 code represents the demoday-ready
integration of:

- Deterministic camera-based traffic light perception
- Vehicle-to-Infrastructure (V2I) UDP signal reception from an ESP32 controller
- Sensor-fused drive control via PCA9685 PWM on I²C

The system must respond to RED, YELLOW, and GREEN light states, defaulting to a safe STOP
on any uncertainty. No machine learning inference is used; all classification is deterministic
HSV-based.

---

## 3. System Architecture

### 3.1 Module Roles

| File               | Class / Role                          | Responsibility                                      |
|--------------------|---------------------------------------|-----------------------------------------------------|
| `3led_detector.py` | `DeterministicTrafficLightDetector`   | Camera frame → HSV masks → classified signal        |
| `3led_v2i.py`      | `V2IReceiver`                         | UDP socket → threaded JSON parsing → live state     |
| `3led_demoday.py`  | `main()` integration loop             | Camera + V2I → vote → fusion → PWM → log → stream  |

### 3.2 Control Flow (Single Loop Iteration)

```
Camera frame (CSI, GStreamer, 640×480 @ 30fps)
  └─ DeterministicTrafficLightDetector.detect()
       └─ ROI crop → BGR→HSV → RED/YELLOW/GREEN masks → erosion → classify
           └─ raw_state (with confidence)

raw_state → vote_buffer (deque, maxlen=7) → majority_vote() → camera_voted_state

V2IReceiver.get_latest() → v2i_state, v2i_remaining (from ESP32 UDP, port 5005)

Fusion Logic:
  AGREE   : cam == v2i (both known)           → use agreed state
  CAM     : cam confident (≥ 0.80)            → trust camera
  V2I     : cam weak or unknown, V2I live     → trust V2I
  CAM_WEAK: cam known but low confidence      → use camera anyway
  FAILSAFE: all uncertain                     → RED (STOP)

final_state → set_pwm(CH_THROTTLE, ESC_FORWARD | ESC_NEUTRAL)
           → CSV log row
           → MJPEG annotated stream frame
```

### 3.3 Hardware Constants

| Parameter       | Value | Notes                                          |
|-----------------|-------|------------------------------------------------|
| I²C bus         | 1     | Jetson I²C-1                                   |
| PCA9685 address | 0x40  | Standard                                       |
| ESC_NEUTRAL     | 307   | ~1.5ms PWM pulse (stopped)                     |
| ESC_FORWARD     | 330   | ~1.6ms PWM pulse (slow forward, 23-step delta) |
| STEER_CENTER    | file  | Loaded from `/home/jetson/steer_center.txt`    |
| STEER fallback  | 370   | Left limit — calibration risk (see §6.2)       |
| VOTE_N          | 7     | ~233ms at 30fps                                |
| Confidence good | 0.80  | Threshold for CAM authority                    |
| Confidence min  | 0.60  | Detector UNKNOWN gate                          |
| V2I timeout     | 2.0s  | ESP32 packet age limit                         |

---

## 4. Work Performed (0409 Session)

Based on code content, the 0409 session produced:

1. **Perception module** (`3led_detector.py`): Full HSV detector with dual-range RED wrapping,
   configurable ROI, erosion noise filter, confidence scoring, and safety-priority tie-breaking
   (RED > YELLOW > GREEN).

2. **V2I receiver** (`3led_v2i.py`): Thread-safe UDP listener, JSON validation, 2.0s staleness
   timeout with fallback to UNKNOWN, rate-limited console logging (packet #1 and every 150th).

3. **Integration loop** (`3led_demoday.py`): Full demoday loop including PCA9685 init with
   readback verification, ESC arming sequence, GStreamer camera pipeline, 7-frame majority vote,
   4-tier fusion logic, CSV logging with flush every 10 loops, MJPEG stream server on port 8080,
   and SIGINT/SIGTERM safe-stop handler.

---

## 5. Observations and Evidence

### 5.1 Strengths

- **Deterministic perception**: No ML inference dependency. Reproducible classification at every
  run. Fail-transparent (confidence always emitted).

- **Conservative safety doctrine**: YELLOW treated as STOP. FAILSAFE defaults to RED. RED wins
  pixel-count ties by design in both the detector and the vote tie-breaker.

- **PCA9685 readback verification**: PRESCALE and MODE1 registers are verified after init.
  Hardware faults surface immediately rather than silently producing wrong PWM.

- **Majority vote buffer**: 7-frame window eliminates single-frame noise. At 30fps this adds
  ~233ms of latency, acceptable for a traffic light stop/go decision.

- **Fusion has 5 deterministic branches**: Every sensor combination maps to an explicit,
  traceable decision source logged to CSV.

- **CSV log completeness**: Every loop row contains: loop count, timestamp, raw state,
  voted camera state, V2I state, live flag, remaining seconds, final state, source, throttle,
  confidence, and pixel counts for all three colors. Post-run analysis is fully supported.

- **MJPEG stream**: Live annotated view available at `http://<JETSON_IP>:8080` during any run.
  ROI box, all state fields, and throttle status are overlaid.

### 5.2 Observations Requiring Attention

See §6 for full details.

- Camera non-recovery on sustained read failure
- STEER_CENTER fallback lands at left-limit (370)
- `v2i_remaining` not used in fusion timing decisions
- PRINT_EVERY_N_LOOPS=5 may produce excessive terminal output at 30fps
- Log file opened at module import level, not inside `main()`

---

## 6. Decisions Made

### 6.1 YELLOW = STOP

YELLOW is treated identically to RED in both `state_to_target()` and the fusion doctrine. Code
comment: *"until timing+distance arbitration exists."* This is the correct conservative choice for
a first demoday integration where no speed/distance model is active.

### 6.2 Steering Held at Center

Steering is set to STEER_CENTER at init and not modified during the main loop. The car runs straight.
Traffic light response is throttle-only. This is intentional for the SSUNC demoday scope.

### 6.3 No ML Inference

The deterministic HSV approach was chosen over a learned model. This removes inference latency,
eliminates model version drift, and makes the perception pipeline reviewable and auditable without
a GPU.

---

## 7. Current State

The three files form a complete, runnable demoday loop. No missing imports or broken references
were found within the 0409/Dev folder boundary (assuming `ssunc_perception/` is deployed to
`/home/jetson/jetson/ssunc_perception/` as specified in module docstrings).

**Run log evidence**: Two CSV logs exist in `logs/`:
- `run_20260410_111054.csv`
- `run_20260410_112404.csv`

This confirms the system executed on 2026-04-10, the day after the 0409 code was authored.

---

## 8. Risks and Recommendations

### R1 — Camera Non-Recovery (Medium Risk)
**Observation**: On `cap.read()` failure, the loop retries with `time.sleep(0.05)` indefinitely
but never re-opens the capture device. A fully dropped camera will loop forever with WARN messages
while the throttle state is unchanged.
**Recommendation**: Add a consecutive-failure counter (e.g., 30 failures ≈ 1.5s) that calls
`safe_stop()`.

### R2 — STEER_CENTER Fallback = Left Limit (High Risk)
**Observation**: If `/home/jetson/steer_center.txt` is absent, `STEER_CENTER` falls back to 370,
which equals `STEER_LEFT_LIMIT`. The car would be steered hard-left at startup.
**Recommendation**: Change fallback to a neutral mid-range value (e.g., 325 or average of 280
and 370 = 325), or raise an exception if the file is missing rather than silently using a
rail-limit value.

### R3 — V2I Remaining Unused in Fusion (Low Risk / Feature Gap)
**Observation**: `v2i_remaining` is logged and displayed but not used in the fusion decision.
A GREEN signal with <0.5s remaining creates a window where the car accelerates just before the
light transitions to RED.
**Recommendation**: Add a `v2i_remaining` guard: if V2I is live, state is GREEN, and
`v2i_remaining < THRESHOLD_S`, treat as YELLOW (STOP). This is noted in the code as future work.

### R4 — Terminal Verbosity at 30fps (Low Risk)
**Observation**: `PRINT_EVERY_N_LOOPS = 5` causes ~6 print calls per second even without state
changes. This can be noisy in terminal sessions and may marginally affect loop timing on slower
serial connections.
**Recommendation**: Raise to 30 (≈1 print/sec) for steady-state monitoring, or gate prints to
state-change events only.

### R5 — Log File at Module Level (Low Risk)
**Observation**: The CSV log file is created and opened at lines 68–86, outside `main()`. If the
module is imported in a test context, a log file is created immediately.
**Recommendation**: Move log file creation inside `main()` or guard with `if __name__ == '__main__'`.

---

## 9. Next Steps

1. **Verify calibration**: Confirm `/home/jetson/steer_center.txt` exists on the target Jetson
   and contains a verified center value before any run.

2. **Add camera recovery**: Implement consecutive-fail counter → `safe_stop()` in the main loop.

3. **Review run logs**: Analyze `run_20260410_111054.csv` and `run_20260410_112404.csv` for
   decision source distribution (AGREE / CAM / V2I / FAILSAFE ratios) to assess sensor fusion
   effectiveness during the April 10 runs.

4. **Consider V2I remaining guard**: Evaluate whether SSUNC scoring penalizes entering an
   intersection on a late GREEN. If so, implement the remaining-time threshold in fusion.

5. **ROI tuning**: Confirm the hardcoded ROI `(240, 100) → (400, 300)` is aligned with the
   physical camera mount. If the camera angle changes between runs, the ROI must be updated.

6. **Promote to ssunc_perception/**: The stable versions of `3led_detector.py` and `3led_v2i.py`
   appear to have counterparts in `ssunc_perception/traffic_light_detector.py` and
   `ssunc_perception/v2i_receiver.py`. Confirm 0409/Dev versions are in sync with or supersede
   those deployed files.

---

## 10. Assumptions and Missing Information

- Actual run performance (accuracy, false-positive rate, timing) is not derivable from code alone.
  Run log analysis (next step #3) is needed for quantitative validation.
- Participant count and competition result for the April 10 runs are not documented here.
- HSV threshold calibration history is unknown. Current thresholds are engineering estimates;
  no documented calibration runs are referenced in 0409 files.
- `ssunc_perception/` deployed state vs. 0409/Dev state has not been diff-verified in this record.
