#!/usr/bin/env python3
"""
demoday.py — ACDC JetRacer Pro
Networked Autonomy — Urban Track

Pipeline each loop tick:
  Camera frame
    ├─ DeterministicTrafficLightDetector  → signal + confidence
    └─ LaneDetector                       → lane offset (px)
  V2IReceiver                             → UDP state from ESP32
  majority_vote (7-frame window)          → camera_voted_state
  fuse()                                  → final_state + decision_source
  Throttle: GREEN → ESC_FORWARD, else ESC_NEUTRAL
  Steering: GREEN → PIDSteering → us_to_ticks → PCA9685
            STOP  → pid.reset(), hold STEER_CENTER

Fusion Doctrine (AGREE-first — v3):
  V2I live (packet age ≤ 2.0 s):
    camera == GREEN AND v2i == GREEN  →  GO   (AGREE)
    any mismatch                      →  more-restrictive state (STOP)
  V2I offline (packet age > 2.0 s):
    camera known + confident          →  camera decides  (CAM_FALLBACK)
    camera known, low confidence      →  camera decides  (CAM_WEAK_FALLBACK)
    camera unknown                    →  RED             (FAILSAFE)

Deploy layout (Jetson and local repo are identical):
  demoday.py               ← this file, run from here
  ssunc_perception/
    traffic_light_detector.py
    v2i_receiver.py
    lane_detect.py
    pid_steer.py
  logs/                    ← auto-created on first run
"""

import collections
import csv
import os
import signal
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import smbus2

# ─── Module path — works from repo root and from /home/jetson/ ───────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, 'ssunc_perception'))

from traffic_light_detector import DeterministicTrafficLightDetector
from v2i_receiver import V2IReceiver
from lane_detect import LaneDetector
from pid_steer import PIDSteering, STEER_CENTER_US

# ─── Hardware constants ───────────────────────────────────────────────────────
BUS       = 1
ADDR      = 0x40
MODE1     = 0x00
PRESCALE  = 0xFE
LED0_ON_L = 0x06

CH_STEER    = 0
CH_THROTTLE = 1

ESC_NEUTRAL      = 307   # PCA9685 ticks ≈ 1499 µs  (motor stopped)
ESC_FORWARD      = 330   # PCA9685 ticks ≈ 1611 µs  (normal creep speed)
ESC_FORWARD_FAST = 345   # PCA9685 ticks ≈ 1685 µs  (rush speed — clear intersection quickly)
                         # Tune this: increase if car doesn't clear in time,
                         # decrease if car becomes hard to control at speed.

# Steering — hardcoded from hardware constants table, no file dependency
STEER_CENTER      = 320   # ticks ≈ 1563 µs  (straight ahead)
STEER_RIGHT_LIMIT = 280   # ticks ≈ 1367 µs  (right soft stop)
STEER_LEFT_LIMIT  = 370   # ticks ≈ 1807 µs  (left soft stop)

STREAM_HOST = '0.0.0.0'
STREAM_PORT = 8080

# ─── Loop tuning ─────────────────────────────────────────────────────────────
VOTE_N              = 7     # camera majority-vote window (frames ≈ 233 ms @ 30fps)
CAMERA_CONF_GOOD    = 0.80  # confidence threshold for camera authority in V2I-offline mode
PRINT_EVERY_N_LOOPS = 30    # ≈ 1 heartbeat print/sec at 30fps

# ─── Physics-based clearance model ───────────────────────────────────────────
#
# PHYSICS_MODE = True  → use kinematic dead-reckoning to decide speed/stop.
# PHYSICS_MODE = False → fall back to threshold heuristic (safe pre-calibration).
#
# To enable physics mode, measure all four constants on the real ASVP track:
#
#   DIST_TO_STOP_LINE_M  : from car nose (waiting position) to the stop line (m).
#                          Measure with a tape when the car is in its start gate.
#
#   DIST_INTERSECTION_M  : distance from stop line to the far edge of the
#                          intersection the car must fully cross (m).
#
#   SPEED_AT_FORWARD_MPS : m/s at ESC_FORWARD (330 ticks).
#                          Time the car over a 1 m straight, 3 runs, average.
#                          SPEED = 1.0 / avg_seconds
#
#   SPEED_AT_FAST_MPS    : same procedure at ESC_FORWARD_FAST (345 ticks).
#
# Clearance logic (when PHYSICS_MODE = True):
#   t_to_reach  = d_remaining / SPEED_AT_FORWARD_MPS
#   t_to_clear_normal = (d_remaining + DIST_INTERSECTION_M) / SPEED_AT_FORWARD_MPS
#   t_to_clear_fast   = (d_remaining + DIST_INTERSECTION_M) / SPEED_AT_FAST_MPS
#
#   v2i_remaining > t_to_clear_normal  → NORMAL   (clears with time to spare)
#   v2i_remaining > t_to_clear_fast    → RUSH     (needs fast to clear)
#   v2i_remaining > t_to_reach         → PHYSICS_STOP (enters but can't clear)
#   v2i_remaining ≤ t_to_reach         → PHYSICS_STOP (can't even reach line)
#
PHYSICS_MODE = False          # set True only after all four constants are measured

DIST_TO_STOP_LINE_M   = 1.5   # m  ← MEASURE on real track before enabling physics mode
DIST_INTERSECTION_M   = 0.6   # m  ← MEASURE on real track before enabling physics mode
SPEED_AT_FORWARD_MPS  = 0.20  # m/s ← CALIBRATE: time 1 m run at ESC_FORWARD
SPEED_AT_FAST_MPS     = 0.28  # m/s ← CALIBRATE: time 1 m run at ESC_FORWARD_FAST

# ─── Threshold fallback (active when PHYSICS_MODE = False) ───────────────────
# Used pre-calibration. Approximates the physics model with fixed time gates.
# These are intentionally conservative; tighten after confirmed physics runs.
V2I_STOP_THRESHOLD_S = 1.5   # s — do NOT enter intersection below this
V2I_RUSH_THRESHOLD_S = 3.0   # s — switch to fast speed below this

# ─── Logging ─────────────────────────────────────────────────────────────────
LOG_DIR = os.path.join(_HERE, 'logs')
os.makedirs(LOG_DIR, exist_ok=True)

# ─── Global handles (needed by safe_stop signal handler) ─────────────────────
_stream_lock   = threading.Lock()
_stream_frame  = None
_stream_server = None
_cap_ref       = None
_shutdown_called = False
_log_file      = None
_log_writer    = None
_log_filename  = None
v2i            = None
bus            = smbus2.SMBus(BUS)


# ─── Utilities ────────────────────────────────────────────────────────────────

def us_to_ticks(us: float) -> int:
    """Convert microseconds to PCA9685 raw ticks (50 Hz, prescale=121, period=20 ms)."""
    return int(round(us * 4096 / 20_000))


def safety_rank(state: str) -> int:
    return {"RED": 3, "YELLOW": 2, "GREEN": 1, "UNKNOWN": 0}.get(state, 0)


def restrictive(a: str, b: str) -> str:
    """Return the more restrictive of two states. UNKNOWN is treated as RED."""
    def coerce(s):
        return s if s != "UNKNOWN" else "RED"
    ca, cb = coerce(a), coerce(b)
    return ca if safety_rank(ca) >= safety_rank(cb) else cb


def majority_vote(vote_buffer) -> str:
    counts = {s: vote_buffer.count(s) for s in ("RED", "YELLOW", "GREEN", "UNKNOWN")}
    best_state, best_count, best_rank = "UNKNOWN", -1, -1
    for state, count in counts.items():
        rank = safety_rank(state)
        if count > best_count or (count == best_count and rank > best_rank):
            best_state, best_count, best_rank = state, count, rank
    return best_state


def state_to_throttle(state: str) -> int:
    return ESC_FORWARD if state == "GREEN" else ESC_NEUTRAL


def speed_for_green(v2i_remaining: float, v2i_live: bool,
                    d_remaining: Optional[float] = None):
    """
    Given AGREE-GREEN, decide throttle and mode based on time remaining
    and (optionally) estimated distance to the stop line.

    Returns (throttle_ticks, throttle_mode) where throttle_mode is one of:
      NORMAL        — plenty of time, enter at standard creep speed
      RUSH          — light expiring, boost to clear intersection before red
      LATE_STOP     — threshold fallback: too late to safely enter (pre-calibration)
      PHYSICS_STOP  — kinematic check: cannot clear intersection before red

    Special values:
      v2i_remaining >= 999.0 → ESP32 MANUAL hold, no transition coming → NORMAL
      v2i_remaining <= 0.0   → unknown/stale → NORMAL (camera authority preserved)
      d_remaining is None    → distance not yet tracked (first GO tick) → NORMAL

    Physics mode (PHYSICS_MODE = True, all constants calibrated):
      Uses dead-reckoning distance and measured speeds to make an exact
      kinematic clearance decision instead of fixed time thresholds.

    Threshold mode (PHYSICS_MODE = False, default):
      Uses V2I_STOP_THRESHOLD_S and V2I_RUSH_THRESHOLD_S as conservative gates.
      Active until all four track constants are measured and verified.
    """
    if not v2i_live:
        return ESC_FORWARD, "NORMAL"
    if v2i_remaining >= 999.0 or v2i_remaining <= 0.0:
        return ESC_FORWARD, "NORMAL"

    if PHYSICS_MODE and d_remaining is not None:
        # ── Kinematic clearance check ──────────────────────────────────────
        # Use conservative (slower) speed for the reach estimate so we never
        # optimistically enter when we can't clear.
        t_to_reach        = d_remaining / SPEED_AT_FORWARD_MPS
        t_to_clear_normal = (d_remaining + DIST_INTERSECTION_M) / SPEED_AT_FORWARD_MPS
        t_to_clear_fast   = (d_remaining + DIST_INTERSECTION_M) / SPEED_AT_FAST_MPS

        if v2i_remaining >= t_to_clear_normal:
            return ESC_FORWARD,      "NORMAL"        # clears at normal speed
        elif v2i_remaining >= t_to_clear_fast:
            return ESC_FORWARD_FAST, "RUSH"          # needs fast speed to clear
        elif v2i_remaining >= t_to_reach:
            return ESC_NEUTRAL,      "PHYSICS_STOP"  # enters but can't clear — stop now
        else:
            return ESC_NEUTRAL,      "PHYSICS_STOP"  # can't even reach the line in time
    else:
        # ── Threshold fallback (pre-calibration) ──────────────────────────
        if v2i_remaining <= V2I_STOP_THRESHOLD_S:
            return ESC_NEUTRAL,      "LATE_STOP"
        if v2i_remaining <= V2I_RUSH_THRESHOLD_S:
            return ESC_FORWARD_FAST, "RUSH"
        return ESC_FORWARD,          "NORMAL"


# ─── Fusion (AGREE-first doctrine) ───────────────────────────────────────────

def fuse(camera_voted: str, v2i_state: str, v2i_live: bool,
         cam_known: bool, cam_confident: bool):
    """
    Returns (final_state, decision_source).

    V2I live path:   BOTH must agree GREEN to move.
                     Any mismatch → more-restrictive state.
    V2I offline path: Camera is sole authority.
    """
    if v2i_live:
        if camera_voted == "GREEN" and v2i_state == "GREEN":
            return "GREEN", "AGREE"

        merged = restrictive(camera_voted, v2i_state)

        if camera_voted == "UNKNOWN" and v2i_state != "UNKNOWN":
            return merged, "V2I_DOMINANT"
        if v2i_state == "UNKNOWN" and camera_voted != "UNKNOWN":
            return merged, "CAM_DOMINANT"
        return merged, "DISAGREE_SAFE"

    else:
        # V2I offline — camera-only fallback
        if cam_known and cam_confident:
            return camera_voted, "CAM_FALLBACK"
        if cam_known:
            return camera_voted, "CAM_WEAK_FALLBACK"
        return "RED", "FAILSAFE"


# ─── MJPEG stream ─────────────────────────────────────────────────────────────

def update_stream_frame(annotated_bgr):
    global _stream_frame
    ok, buf = cv2.imencode('.jpg', annotated_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        with _stream_lock:
            _stream_frame = buf.tobytes()


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                with _stream_lock:
                    frame_bytes = _stream_frame
                if frame_bytes is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                self.wfile.write(frame_bytes)
                self.wfile.write(b'\r\n')
                time.sleep(0.033)
        except (BrokenPipeError, ConnectionResetError):
            pass


def start_stream_server():
    global _stream_server
    _stream_server = HTTPServer((STREAM_HOST, STREAM_PORT), MJPEGHandler)
    t = threading.Thread(target=_stream_server.serve_forever, daemon=True)
    t.start()
    print(f"[stream] MJPEG at http://<JETSON_IP>:{STREAM_PORT}")


# ─── I²C / PCA9685 ────────────────────────────────────────────────────────────

def w(reg, val):
    bus.write_byte_data(ADDR, reg, val & 0xFF)

def r(reg):
    return bus.read_byte_data(ADDR, reg)

def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])

def init_pca9685():
    w(MODE1, 0x00); time.sleep(0.1)
    w(MODE1, 0x10); time.sleep(0.1)
    w(PRESCALE, 121); time.sleep(0.1)
    w(MODE1, 0xA1); time.sleep(0.1)
    mode1_val    = r(MODE1)
    prescale_val = r(PRESCALE)
    print(f"[init] MODE1={hex(mode1_val)} PRESCALE={prescale_val}")
    if prescale_val != 121:
        raise RuntimeError(f"[init] PRESCALE readback wrong ({prescale_val})")
    if mode1_val & 0x10:
        raise RuntimeError(f"[init] Chip still in sleep mode (MODE1={hex(mode1_val)})")
    print("[init] PCA9685 verified OK.")


# ─── Safe stop ────────────────────────────────────────────────────────────────

def safe_stop(sig=None, frame=None):
    global _shutdown_called
    if _shutdown_called:
        sys.exit(0)
    _shutdown_called = True
    print("\n[safe_stop] Stopping car.")
    try:
        set_pwm(CH_THROTTLE, ESC_NEUTRAL)
        set_pwm(CH_STEER,    STEER_CENTER)
        time.sleep(0.5)
    except Exception as e:
        print(f"[safe_stop] I2C error: {e}")
    finally:
        try:
            if v2i is not None:
                v2i.stop()
        except Exception as exc:
            print(f"[safe_stop] V2I: {exc}")
        try:
            if _cap_ref is not None and _cap_ref.isOpened():
                _cap_ref.release()
        except Exception as exc:
            print(f"[safe_stop] camera: {exc}")
        try:
            if _stream_server is not None:
                _stream_server.shutdown()
                _stream_server.server_close()
        except Exception as exc:
            print(f"[safe_stop] stream: {exc}")
        try:
            if _log_file is not None:
                _log_file.flush()
                _log_file.close()
                print(f"[safe_stop] Log saved: {_log_filename}")
        except Exception as exc:
            print(f"[safe_stop] log: {exc}")
        try:
            bus.close()
        except Exception as exc:
            print(f"[safe_stop] I2C: {exc}")
    sys.exit(0)


signal.signal(signal.SIGINT,  safe_stop)
signal.signal(signal.SIGTERM, safe_stop)


# ─── Camera ───────────────────────────────────────────────────────────────────

def gstreamer_pipeline():
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


# ─── Annotation ───────────────────────────────────────────────────────────────

def annotate_frame(base_frame, lane_debug,
                   detector, camera_voted, v2i_state, final_state,
                   result, target_throttle, source, v2i_remaining,
                   lane_offset, steer_ticks):
    """
    Overlay traffic-light HUD on top of the lane-debug frame.
    Lane debug (from LaneDetector._last_debug) already shows Hough lines,
    L/R/Y markers, and lane-center estimate in the bottom 45%.
    """
    out = lane_debug.copy() if lane_debug is not None else base_frame.copy()

    x1, y1 = detector.roi_top_left
    x2, y2 = detector.roi_bottom_right

    color_map = {
        "GREEN": (0, 255, 0), "YELLOW": (0, 255, 255),
        "RED":   (0, 0, 255), "UNKNOWN": (200, 200, 200),
    }
    color = color_map.get(final_state, (200, 200, 200))

    cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)

    throttle_str = "FWD" if target_throttle == ESC_FORWARD else "STOP"
    offset_str   = f"{lane_offset:+.1f}" if lane_offset is not None else "None"

    lines = [
        (f"raw={result.signal:7s} cam={camera_voted:7s} "
         f"v2i={v2i_state:7s} final={final_state:7s}"),
        (f"src={source} rem={v2i_remaining:.2f}s conf={result.confidence:.2f}"),
        (f"R={result.red_pixels} Y={result.yellow_pixels} G={result.green_pixels} "
         f"{throttle_str}({target_throttle})"),
        (f"lane_off={offset_str}px  steer={steer_ticks}tks"),
    ]
    y_pos = 25
    for i, line in enumerate(lines):
        col = color if i == 0 else (255, 255, 255)
        cv2.putText(out, line, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.50, col, 2)
        y_pos += 22

    return out


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    global _cap_ref, v2i, _log_file, _log_writer, _log_filename

    start_stream_server()

    # ── PCA9685 ──────────────────────────────────────────────────────────────
    print("[main] Initialising PCA9685...")
    init_pca9685()

    print(f"[main] Arming ESC. STEER_CENTER={STEER_CENTER} ticks. Holding neutral 2 s...")
    set_pwm(CH_THROTTLE, ESC_NEUTRAL)
    set_pwm(CH_STEER,    STEER_CENTER)
    time.sleep(1.0)
    set_pwm(CH_STEER, STEER_CENTER)
    time.sleep(1.0)
    print("[main] ESC armed.")

    # ── Camera ───────────────────────────────────────────────────────────────
    print("[main] Opening camera...")
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    _cap_ref = cap
    if not cap.isOpened():
        print("[ERROR] Camera failed to open. Try: sudo systemctl restart nvargus-daemon")
        safe_stop()
    print("[main] Camera opened.")

    # ── Perception modules ───────────────────────────────────────────────────
    detector = DeterministicTrafficLightDetector(
        roi_top_left=(240, 100),
        roi_bottom_right=(400, 300),
    )
    lane_det = LaneDetector(frame_w=640, frame_h=480)
    pid      = PIDSteering(steer_center=STEER_CENTER_US)   # 1563 µs = 320 ticks

    # ── V2I ──────────────────────────────────────────────────────────────────
    v2i = V2IReceiver()
    v2i.start()

    # ── CSV log ──────────────────────────────────────────────────────────────
    _log_filename = os.path.join(LOG_DIR, f"run_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    _log_file     = open(_log_filename, 'w', newline='')
    _log_writer   = csv.writer(_log_file)
    _log_writer.writerow([
        'loop', 'timestamp',
        'raw_state', 'camera_voted', 'v2i_state', 'v2i_live', 'v2i_remaining',
        'final_state', 'decision_source',
        'throttle', 'throttle_mode', 'd_remaining_m', 'confidence',
        'red_px', 'yellow_px', 'green_px',
        'lane_offset', 'steer_ticks',
    ])

    print("[main] All modules ready. Entering control loop.")
    print(f"[main] TL ROI: {detector.roi_top_left} → {detector.roi_bottom_right}")
    print(f"[main] Vote window: {VOTE_N} frames | NEUTRAL={ESC_NEUTRAL} FWD={ESC_FORWARD}")
    print(f"[main] STEER_CENTER={STEER_CENTER} ticks | limits {STEER_RIGHT_LIMIT}–{STEER_LEFT_LIMIT}")
    print(f"[main] Fusion: AGREE required for GO. CAM fallback when V2I offline only.")
    print(f"[main] Stream: http://<JETSON_IP>:{STREAM_PORT}")
    print("[main] Ctrl+C to stop safely.\n")

    vote_buffer    = collections.deque(maxlen=VOTE_N)
    cur_throttle   = ESC_NEUTRAL
    cur_steer      = STEER_CENTER
    cur_steer_ticks = STEER_CENTER
    stable_state   = "UNKNOWN"
    last_source    = "INIT"
    loop_count     = 0
    cam_fail_count = 0
    CAM_FAIL_LIMIT = 30   # ~1 s of consecutive failures → safe stop

    # ── Dead-reckoning distance tracker ──────────────────────────────────────
    # Tracks elapsed time in the current GO window to estimate distance covered.
    # Resets each time final_state transitions into GREEN from a non-GREEN state.
    # d_remaining = DIST_TO_STOP_LINE_M - (elapsed * SPEED_AT_FORWARD_MPS)
    # Active only when PHYSICS_MODE = True; otherwise passed as None.
    _prev_was_green = False
    _go_start_time  = None

    try:
        while True:
            # ── Frame capture ─────────────────────────────────────────────
            ret, frame = cap.read()
            if not ret:
                cam_fail_count += 1
                print(f"[WARN] Camera read failed ({cam_fail_count}/{CAM_FAIL_LIMIT})")
                if cam_fail_count >= CAM_FAIL_LIMIT:
                    print("[ERROR] Camera unrecoverable — stopping.")
                    safe_stop()
                time.sleep(0.05)
                continue
            cam_fail_count = 0

            # ── Lane detection ────────────────────────────────────────────
            lane_offset, left_x, right_x, yellow_x = lane_det.get_offset(frame)

            # ── Traffic light detection ───────────────────────────────────
            result    = detector.detect(frame)
            raw_state = result.signal
            vote_buffer.append(raw_state)
            camera_voted = majority_vote(vote_buffer)

            # ── V2I ───────────────────────────────────────────────────────
            v2i_state, v2i_remaining = v2i.get_latest()
            v2i_live = v2i.is_live

            cam_known     = camera_voted != "UNKNOWN"
            cam_confident = result.confidence >= CAMERA_CONF_GOOD

            # ── Fusion ────────────────────────────────────────────────────
            final_state, decision_source = fuse(
                camera_voted, v2i_state, v2i_live,
                cam_known, cam_confident,
            )

            # ── Dead-reckoning distance estimate ──────────────────────────
            if final_state == "GREEN":
                if not _prev_was_green:
                    # New GO window — reset approach timer
                    _go_start_time  = time.monotonic()
                    _prev_was_green = True
                if PHYSICS_MODE and _go_start_time is not None:
                    elapsed_go   = time.monotonic() - _go_start_time
                    d_covered    = elapsed_go * SPEED_AT_FORWARD_MPS   # conservative estimate
                    d_remaining  = max(0.0, DIST_TO_STOP_LINE_M - d_covered)
                else:
                    d_remaining  = None
            else:
                _prev_was_green = False
                _go_start_time  = None
                d_remaining     = None

            # ── Throttle ──────────────────────────────────────────────────
            if final_state == "GREEN":
                target_throttle, throttle_mode = speed_for_green(
                    v2i_remaining, v2i_live, d_remaining)
                # LATE_STOP / PHYSICS_STOP overrides fusion: don't enter intersection
                if target_throttle == ESC_NEUTRAL:
                    final_state     = "RED"
                    decision_source = decision_source + "+" + throttle_mode
            else:
                target_throttle = ESC_NEUTRAL
                throttle_mode   = "STOP"
            if target_throttle != cur_throttle:
                set_pwm(CH_THROTTLE, target_throttle)
                cur_throttle = target_throttle

            # ── Steering (PID when GO, center + reset when STOP) ─────────
            if final_state == "GREEN":
                steer_us    = pid.compute(lane_offset)   # None → returns center
                steer_ticks = us_to_ticks(steer_us)
                steer_ticks = max(STEER_RIGHT_LIMIT, min(STEER_LEFT_LIMIT, steer_ticks))
                if steer_ticks != cur_steer:
                    set_pwm(CH_STEER, steer_ticks)
                    cur_steer = steer_ticks
            else:
                pid.reset()
                if cur_steer != STEER_CENTER:
                    set_pwm(CH_STEER, STEER_CENTER)
                    cur_steer = STEER_CENTER
                steer_ticks = STEER_CENTER

            # ── Heartbeat print ───────────────────────────────────────────
            state_changed = (final_state != stable_state or decision_source != last_source)
            if state_changed or (loop_count % PRINT_EVERY_N_LOOPS == 0):
                off_str = f"{lane_offset:+.1f}" if lane_offset is not None else "None"
                d_str   = f"{d_remaining:.2f}m" if d_remaining is not None else ("PHY" if PHYSICS_MODE else "---")
                print(
                    f"[{loop_count:06d}] "
                    f"raw={raw_state:7s} cam={camera_voted:7s} "
                    f"v2i={v2i_state:7s} live={str(v2i_live):5s} rem={v2i_remaining:.2f}s | "
                    f"final={final_state:7s} src={decision_source:28s} spd={throttle_mode:12s} d={d_str} | "
                    f"conf={result.confidence:.2f} lane={off_str}px steer={steer_ticks}tks"
                )

            stable_state = final_state
            last_source  = decision_source

            # ── CSV log ───────────────────────────────────────────────────
            _log_writer.writerow([
                loop_count,
                f"{result.timestamp:.4f}",
                raw_state, camera_voted,
                v2i_state, int(v2i_live), f"{v2i_remaining:.2f}",
                final_state, decision_source,
                target_throttle, throttle_mode,
                f"{d_remaining:.3f}" if d_remaining is not None else "",
                f"{result.confidence:.4f}",
                result.red_pixels, result.yellow_pixels, result.green_pixels,
                f"{lane_offset:.2f}" if lane_offset is not None else "",
                steer_ticks,
            ])
            if loop_count % 10 == 0:
                _log_file.flush()

            # ── MJPEG stream ──────────────────────────────────────────────
            annotated = annotate_frame(
                frame, lane_det._last_debug,
                detector, camera_voted, v2i_state, final_state,
                result, target_throttle, decision_source, v2i_remaining,
                lane_offset, steer_ticks,
            )
            update_stream_frame(annotated)
            loop_count += 1

    except KeyboardInterrupt:
        print("[main] KeyboardInterrupt.")
        safe_stop()
    except Exception as e:
        print(f"[ERROR] Unhandled exception: {e}")
        import traceback; traceback.print_exc()
        safe_stop()

    safe_stop()


if __name__ == '__main__':
    main()
