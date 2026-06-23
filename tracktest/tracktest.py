#!/usr/bin/env python3
"""
tracktest.py  —  JPA Track Test Harness
========================================
Camera-only stop/go + PID steering for straight-line and lap testing.
NO V2I. NO ESP32 dependency. Camera is the sole authority.

PURPOSE:
  Validate PID steering and lane detection on the real track without
  requiring the traffic light network. Use this to:
    - Tune Kp / Ki / Kd on the actual course
    - Verify yellow (left) / white (right) boundary detection
    - Confirm the car stays in lane through a full lap
    - Gather calibration data (speed, distance) for 0423/demoday.py physics mode

BEHAVIOR:
  GREEN  -> ESC_FORWARD + PID steering from lane offset
  YELLOW -> ESC_NEUTRAL + steer center (STOP — conservative)
  RED    -> ESC_NEUTRAL + steer center (STOP)
  UNKNOWN -> ESC_NEUTRAL + steer center (FAILSAFE STOP)

DECISION SOURCE:
  Camera-only majority vote over VOTE_N=7 frames.
  No V2I fusion. No AGREE doctrine. Camera alone decides.

LANE BOUNDARIES (ASVP track, right lane):
  Left  boundary : YELLOW tape (center divider)
  Right boundary : WHITE  tape (outer edge)

HARDWARE:
  PCA9685 on I2C bus 1, address 0x40, prescale 121 (50 Hz)
  CH_STEER    = 0
  CH_THROTTLE = 1
  ESC_NEUTRAL = 307 ticks
  ESC_FORWARD = 330 ticks
  STEER_CENTER = 320 ticks

LOGGING:
  CSV written to tracktest/logs/run_YYYYMMDD_HHMMSS.csv
  Columns: loop, timestamp, raw_state, voted_state, final_state,
           throttle, confidence, red_px, yellow_px, green_px,
           steer_ticks, pid_offset

MJPEG stream:
  http://<JETSON_IP>:8080  (traffic light ROI + lane overlay)

DEPLOY:
  scp -r ~/Desktop/JPA/tracktest jetson@172.20.10.13:~/
  ssh jetson@172.20.10.13
  cd ~/tracktest && python3 tracktest.py

Ctrl+C to stop safely.
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

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'perception'))
from traffic_light_detector import DeterministicTrafficLightDetector
from lane_detect_track import LaneDetectorTrack
from pid_steer import PIDSteering, STEER_CENTER_US

# ─── Hardware constants ───────────────────────────────────────────────────────
BUS  = 1
ADDR = 0x40

MODE1    = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

CH_STEER    = 0
CH_THROTTLE = 1

ESC_NEUTRAL = 307   # 1500us  — stopped / armed
ESC_FORWARD = 330   # ~1611us — normal driving speed

STEER_CENTER      = 320   # ticks — straight ahead
STEER_RIGHT_LIMIT = 280   # ticks — max right
STEER_LEFT_LIMIT  = 370   # ticks — max left

# ─── Control parameters ───────────────────────────────────────────────────────
VOTE_N                 = 7      # majority-vote window (frames)
CAMERA_CONFIDENCE_GOOD = 0.80   # below this → UNKNOWN treated as low-confidence
PRINT_EVERY_N_LOOPS    = 5      # heartbeat terminal print interval

# ─── Stream ───────────────────────────────────────────────────────────────────
STREAM_HOST = '0.0.0.0'
STREAM_PORT = 8080

# ─── Logging ──────────────────────────────────────────────────────────────────
_LOG_DIR = os.path.join(os.path.dirname(__file__), 'logs')
os.makedirs(_LOG_DIR, exist_ok=True)
_log_filename = os.path.join(_LOG_DIR, f"run_{time.strftime('%Y%m%d_%H%M%S')}.csv")
_log_file     = open(_log_filename, 'w', newline='')
_log_writer   = csv.writer(_log_file)
_log_writer.writerow([
    'loop', 'timestamp',
    'raw_state', 'voted_state', 'final_state',
    'throttle', 'confidence',
    'red_px', 'yellow_px', 'green_px',
    'steer_ticks', 'pid_offset',
    'yellow_x', 'white_x',
])

# ─── Globals ──────────────────────────────────────────────────────────────────
_stream_lock   = threading.Lock()
_stream_frame  = None
_stream_server = None
_cap_ref       = None
_shutdown_called = False

bus = smbus2.SMBus(BUS)


# ─── I2C / PCA9685 ────────────────────────────────────────────────────────────

def _w(reg, val):
    bus.write_byte_data(ADDR, reg, val & 0xFF)


def _r(reg):
    return bus.read_byte_data(ADDR, reg)


def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])


def us_to_ticks(us: float) -> int:
    return round(us * 4096 / 20000)


def init_pca9685():
    _w(MODE1, 0x00);  time.sleep(0.1)
    _w(MODE1, 0x10);  time.sleep(0.1)
    _w(PRESCALE, 121);time.sleep(0.1)
    _w(MODE1, 0xA1);  time.sleep(0.1)

    mode1_val    = _r(MODE1)
    prescale_val = _r(PRESCALE)
    print(f"[init] MODE1={hex(mode1_val)} PRESCALE={prescale_val}")

    if prescale_val != 121:
        raise RuntimeError(f"[init] PRESCALE readback wrong ({prescale_val})")
    if mode1_val & 0x10:
        raise RuntimeError(f"[init] Chip still in sleep (MODE1={hex(mode1_val)})")

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
        set_pwm(CH_STEER, STEER_CENTER)
        time.sleep(0.5)
    except Exception as e:
        print(f"[safe_stop] I2C error during stop: {e}")

    try:
        if _cap_ref is not None and _cap_ref.isOpened():
            _cap_ref.release()
    except Exception:
        pass
    try:
        if _stream_server is not None:
            _stream_server.shutdown()
            _stream_server.server_close()
    except Exception:
        pass
    try:
        _log_file.flush()
        _log_file.close()
        print(f"[safe_stop] Log saved: {_log_filename}")
    except Exception:
        pass
    try:
        bus.close()
    except Exception:
        pass

    sys.exit(0)


signal.signal(signal.SIGINT,  safe_stop)
signal.signal(signal.SIGTERM, safe_stop)


# ─── Camera pipeline ──────────────────────────────────────────────────────────

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


# ─── Majority vote ────────────────────────────────────────────────────────────

def _safety_rank(state):
    return {"RED": 3, "YELLOW": 2, "GREEN": 1, "UNKNOWN": 0}.get(state, 0)


def majority_vote(vote_buffer):
    counts = {s: vote_buffer.count(s)
              for s in ("RED", "YELLOW", "GREEN", "UNKNOWN")}
    best, best_count, best_rank = "UNKNOWN", -1, -1
    for state, count in counts.items():
        rank = _safety_rank(state)
        if count > best_count or (count == best_count and rank > best_rank):
            best, best_count, best_rank = state, count, rank
    return best


# ─── MJPEG stream ─────────────────────────────────────────────────────────────

def update_stream_frame(bgr):
    global _stream_frame
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
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


# ─── Annotation ───────────────────────────────────────────────────────────────

def annotate_frame(frame, detector, voted_state, final_state, result,
                   steer_ticks, pid_offset, yellow_x, white_x, throttle):
    out = frame.copy()

    # Traffic light ROI box
    x1, y1 = detector.roi_top_left
    x2, y2 = detector.roi_bottom_right
    color_map = {"GREEN": (0,255,0), "YELLOW": (0,255,255),
                 "RED": (0,0,255), "UNKNOWN": (200,200,200)}
    cv2.rectangle(out, (x1, y1), (x2, y2), color_map.get(final_state,(200,200,200)), 2)

    # Text overlay
    line1 = (f"raw={result.signal:7s} voted={voted_state:7s} "
             f"final={final_state:7s}")
    line2 = (f"steer={steer_ticks:3d}  offset="
             f"{(f'{pid_offset:+.1f}' if pid_offset is not None else 'None'):>7}"
             f"  throttle={'FWD' if throttle == ESC_FORWARD else 'STOP'}({throttle})")
    line3 = (f"conf={result.confidence:.2f}  "
             f"R={result.red_pixels} Y={result.yellow_pixels} G={result.green_pixels}")

    col = color_map.get(final_state, (200,200,200))
    cv2.putText(out, line1, (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.50, col, 2)
    cv2.putText(out, line2, (10, 54),  cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255,255,255), 1)
    cv2.putText(out, line3, (10, 76),  cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255,255,255), 1)

    return out


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    global _cap_ref

    start_stream_server()

    print("[main] Initialising PCA9685...")
    init_pca9685()

    print(f"[main] Arming ESC. STEER_CENTER={STEER_CENTER}. Holding neutral 2 s...")
    set_pwm(CH_THROTTLE, ESC_NEUTRAL)
    set_pwm(CH_STEER, STEER_CENTER)
    time.sleep(2.0)
    print("[main] ESC armed.")

    print("[main] Opening camera...")
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    _cap_ref = cap
    if not cap.isOpened():
        print("[ERROR] Camera failed to open.")
        print("[ERROR] Try: sudo systemctl restart nvargus-daemon")
        safe_stop()

    print("[main] Camera opened.")

    detector  = DeterministicTrafficLightDetector(
        roi_top_left=(240, 100),
        roi_bottom_right=(400, 300),
    )
    lane_det  = LaneDetectorTrack()
    pid       = PIDSteering(steer_center=STEER_CENTER_US)

    print("[main] Perception stack ready. Entering control loop.")
    print(f"[main] ROI: {detector.roi_top_left} → {detector.roi_bottom_right}")
    print(f"[main] Vote window: {VOTE_N}  |  ESC NEUTRAL={ESC_NEUTRAL}  FORWARD={ESC_FORWARD}")
    print("[main] Logic: GREEN=GO+PID, all else=STOP+center  (camera-only, no V2I)")
    print(f"[main] Stream: http://<JETSON_IP>:{STREAM_PORT}")
    print("[main] Ctrl+C to stop safely.\n")

    vote_buffer      = collections.deque(maxlen=VOTE_N)
    current_throttle = ESC_NEUTRAL
    current_steer    = STEER_CENTER
    stable_state     = "UNKNOWN"
    loop_count       = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Camera read failed — retrying...")
                time.sleep(0.05)
                continue

            # ── Traffic light detection ────────────────────────────────────
            result        = detector.detect(frame)
            raw_state     = result.signal
            vote_buffer.append(raw_state)
            voted_state   = majority_vote(vote_buffer)

            # Camera-only decision (no V2I fusion)
            if voted_state == "GREEN":
                final_state = "GREEN"
            elif voted_state in ("RED", "YELLOW"):
                final_state = voted_state
            elif voted_state == "UNKNOWN":
                final_state = "RED"   # failsafe
            else:
                final_state = "RED"

            # ── Throttle ──────────────────────────────────────────────────
            if final_state == "GREEN":
                target_throttle = ESC_FORWARD
            else:
                target_throttle = ESC_NEUTRAL

            if target_throttle != current_throttle:
                set_pwm(CH_THROTTLE, target_throttle)
                current_throttle = target_throttle

            # ── Steering ──────────────────────────────────────────────────
            if final_state == "GREEN":
                offset, yellow_x, white_x = lane_det.get_offset(frame)
                pid_offset = offset
                steer_us   = pid.compute(offset)
                steer_ticks = us_to_ticks(steer_us)
                steer_ticks = max(STEER_RIGHT_LIMIT,
                                  min(STEER_LEFT_LIMIT, steer_ticks))
            else:
                # Stopped — center steering, reset PID to clear integrator
                pid.reset()
                offset      = None
                yellow_x    = None
                white_x     = None
                pid_offset  = None
                steer_ticks = STEER_CENTER

            if steer_ticks != current_steer:
                set_pwm(CH_STEER, steer_ticks)
                current_steer = steer_ticks

            # ── Heartbeat print ───────────────────────────────────────────
            state_changed = (final_state != stable_state)
            if state_changed or (loop_count % PRINT_EVERY_N_LOOPS == 0):
                off_str = f"{pid_offset:+.1f}" if pid_offset is not None else "None"
                print(
                    f"[{loop_count:06d}] raw={raw_state:7s} voted={voted_state:7s} "
                    f"final={final_state:7s} | "
                    f"thr={'FWD' if target_throttle == ESC_FORWARD else 'STP'}({target_throttle}) "
                    f"steer={steer_ticks:3d} off={off_str:>7} | "
                    f"conf={result.confidence:.2f} "
                    f"R={result.red_pixels:3d} Y={result.yellow_pixels:3d} G={result.green_pixels:3d}"
                )

            stable_state = final_state

            # ── CSV log ───────────────────────────────────────────────────
            _log_writer.writerow([
                loop_count,
                f"{result.timestamp:.4f}",
                raw_state,
                voted_state,
                final_state,
                target_throttle,
                f"{result.confidence:.4f}",
                result.red_pixels,
                result.yellow_pixels,
                result.green_pixels,
                steer_ticks,
                f"{pid_offset:.2f}" if pid_offset is not None else "",
                f"{yellow_x:.1f}"   if yellow_x  is not None else "",
                f"{white_x:.1f}"    if white_x   is not None else "",
            ])

            if loop_count % 10 == 0:
                _log_file.flush()

            # ── Annotated stream frame ────────────────────────────────────
            # Use lane debug overlay when GREEN, raw frame otherwise
            if final_state == "GREEN" and lane_det._last_debug is not None:
                base_frame = lane_det._last_debug
            else:
                base_frame = frame

            annotated = annotate_frame(
                base_frame, detector,
                voted_state, final_state, result,
                steer_ticks, pid_offset, yellow_x, white_x,
                target_throttle,
            )
            update_stream_frame(annotated)

            loop_count += 1

    except KeyboardInterrupt:
        print("[main] KeyboardInterrupt received.")
        safe_stop()
    except Exception as e:
        print(f"[ERROR] Unhandled exception: {e}")
        import traceback
        traceback.print_exc()
        safe_stop()

    safe_stop()


if __name__ == '__main__':
    main()
