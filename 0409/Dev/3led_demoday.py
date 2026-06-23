#!/usr/bin/env python3
"""
demoday_v2.py — ACDC JetRacer Pro integration
Camera → Perception → Vote → V2I Fusion → Throttle → Log

Behavior:
  RED    -> STOP
  YELLOW -> STOP
  GREEN  -> GO

Fusion Doctrine:
  1. If camera and V2I agree -> use agreed state
  2. If camera is confident -> trust camera on mismatch
  3. If camera is weak/unknown and V2I is live -> trust V2I
  4. If both are uncertain -> fail-safe STOP
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

sys.path.insert(0, '/home/jetson/jetson/ssunc_perception')
from traffic_light_detector import DeterministicTrafficLightDetector
from v2i_receiver import V2IReceiver

# ─── Hardware constants ───────────────────────────────────────────────────────
BUS = 1
ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

CH_STEER = 0
CH_THROTTLE = 1

ESC_NEUTRAL = 307
ESC_FORWARD = 330

STEER_RIGHT_LIMIT = 280
STEER_LEFT_LIMIT = 370

STREAM_HOST = '0.0.0.0'
STREAM_PORT = 8080

_STEER_FALLBACK = 370
try:
    with open('/home/jetson/steer_center.txt', 'r') as _f:
        STEER_CENTER = int(_f.read().strip())
    print(f"[config] STEER_CENTER loaded from file: {STEER_CENTER}")
except FileNotFoundError:
    STEER_CENTER = _STEER_FALLBACK
    print(f"[config] steer_center.txt not found — using fallback: {STEER_CENTER}")

VOTE_N = 7
CAMERA_CONFIDENCE_GOOD = 0.80
PRINT_EVERY_N_LOOPS = 5   # heartbeat print so terminal updates continuously

LOG_DIR = '/home/jetson/logs'
os.makedirs(LOG_DIR, exist_ok=True)
_log_filename = os.path.join(LOG_DIR, f"run_{time.strftime('%Y%m%d_%H%M%S')}.csv")
_log_file = open(_log_filename, 'w', newline='')
_log_writer = csv.writer(_log_file)
_log_writer.writerow([
    'loop',
    'timestamp',
    'raw_state',
    'camera_voted_state',
    'v2i_state',
    'v2i_live',
    'v2i_remaining',
    'final_state',
    'decision_source',
    'throttle',
    'confidence',
    'red_px',
    'yellow_px',
    'green_px',
])

_stream_lock = threading.Lock()
_stream_frame = None
_stream_server = None
_cap_ref = None
_shutdown_called = False
v2i = None

bus = smbus2.SMBus(BUS)


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
    print(f"[stream] MJPEG server running — open http://<JETSON_IP>:{STREAM_PORT}")


def w(reg, val):
    bus.write_byte_data(ADDR, reg, val & 0xFF)


def r(reg):
    return bus.read_byte_data(ADDR, reg)


def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])


def init_pca9685():
    w(MODE1, 0x00)
    time.sleep(0.1)
    w(MODE1, 0x10)
    time.sleep(0.1)
    w(PRESCALE, 121)
    time.sleep(0.1)
    w(MODE1, 0xA1)
    time.sleep(0.1)

    mode1_val = r(MODE1)
    prescale_val = r(PRESCALE)
    print(f"[init] MODE1={hex(mode1_val)} PRESCALE={prescale_val}")

    if prescale_val != 121:
        raise RuntimeError(f"[init] PRESCALE readback wrong ({prescale_val})")
    if mode1_val & 0x10:
        raise RuntimeError(f"[init] Chip still in sleep mode (MODE1={hex(mode1_val)})")

    print("[init] PCA9685 verified OK.")


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
        print(f"[safe_stop] I2C error: {e}")
    finally:
        try:
            if v2i is not None:
                v2i.stop()
        except Exception:
            pass
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


signal.signal(signal.SIGINT, safe_stop)
signal.signal(signal.SIGTERM, safe_stop)


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


def safety_rank(state: str) -> int:
    rank = {
        "RED": 3,
        "YELLOW": 2,
        "GREEN": 1,
        "UNKNOWN": 0,
    }
    return rank.get(state, 0)


def majority_vote(vote_buffer):
    counts = {
        "RED": vote_buffer.count("RED"),
        "YELLOW": vote_buffer.count("YELLOW"),
        "GREEN": vote_buffer.count("GREEN"),
        "UNKNOWN": vote_buffer.count("UNKNOWN"),
    }

    best_state = "UNKNOWN"
    best_count = -1
    best_rank = -1

    for state, count in counts.items():
        rank = safety_rank(state)
        if count > best_count or (count == best_count and rank > best_rank):
            best_state = state
            best_count = count
            best_rank = rank

    return best_state


def state_to_target(state: str) -> int:
    if state == "GREEN":
        return ESC_FORWARD
    return ESC_NEUTRAL


def annotate_frame(frame, detector, camera_voted_state, v2i_state, final_state, result, target, source, v2i_remaining):
    x1, y1 = detector.roi_top_left
    x2, y2 = detector.roi_bottom_right

    color_map = {
        "GREEN": (0, 255, 0),
        "YELLOW": (0, 255, 255),
        "RED": (0, 0, 255),
        "UNKNOWN": (255, 255, 255),
    }
    color = color_map.get(final_state, (255, 255, 255))

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    line1 = (
        f"raw={result.signal:7s} cam={camera_voted_state:7s} "
        f"v2i={v2i_state:7s} final={final_state:7s}"
    )
    line2 = (
        f"src={source} rem={v2i_remaining:.2f}s "
        f"conf={result.confidence:.2f}"
    )
    line3 = (
        f"R={result.red_pixels} Y={result.yellow_pixels} G={result.green_pixels} "
        f"throttle={'FWD' if target == ESC_FORWARD else 'STOP'} ({target})"
    )

    cv2.putText(frame, line1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(frame, line2, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    cv2.putText(frame, line3, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    return frame


def main():
    global _cap_ref, v2i

    start_stream_server()

    print("[main] Initialising PCA9685...")
    init_pca9685()

    print(f"[main] Arming ESC. Steering center = {STEER_CENTER}. Holding neutral 2s...")
    set_pwm(CH_THROTTLE, ESC_NEUTRAL)
    set_pwm(CH_STEER, STEER_CENTER)
    time.sleep(1.0)
    set_pwm(CH_STEER, STEER_CENTER)
    time.sleep(1.0)
    print("[main] ESC armed.")

    print("[main] Opening camera...")
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    _cap_ref = cap
    if not cap.isOpened():
        print("[ERROR] Camera failed to open.")
        print("[ERROR] Try: sudo systemctl restart nvargus-daemon")
        safe_stop()

    print("[main] Camera opened.")

    detector = DeterministicTrafficLightDetector(
        roi_top_left=(240, 100),
        roi_bottom_right=(400, 300),
    )

    v2i = V2IReceiver()
    v2i.start()

    print("[main] Detector ready. Entering control loop.")
    print(f"[main] ROI: {detector.roi_top_left} → {detector.roi_bottom_right}")
    print(f"[main] Vote window: {VOTE_N} frames | ESC NEUTRAL={ESC_NEUTRAL} FORWARD={ESC_FORWARD}")
    print(f"[main] Browser: http://<JETSON_IP>:{STREAM_PORT}")
    print("[main] Logic: RED=STOP, YELLOW=STOP, GREEN=GO")
    print("[main] Authority: fused camera + V2I")
    print("[main] Ctrl+C to stop safely.\n")

    vote_buffer = collections.deque(maxlen=VOTE_N)
    current_throttle = ESC_NEUTRAL
    stable_state = "UNKNOWN"
    last_source = "INIT"
    loop_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Camera read failed — retrying...")
                time.sleep(0.05)
                continue

            result = detector.detect(frame)
            raw_state = result.signal

            vote_buffer.append(raw_state)
            camera_voted_state = majority_vote(vote_buffer)

            v2i_state, v2i_remaining = v2i.get_latest()
            v2i_live = v2i.is_live

            cam_known = camera_voted_state != "UNKNOWN"
            cam_confident = result.confidence >= CAMERA_CONFIDENCE_GOOD
            v2i_known = v2i_live and v2i_state != "UNKNOWN"

            # Fusion logic
            if cam_known and v2i_known and camera_voted_state == v2i_state:
                final_state = camera_voted_state
                decision_source = "AGREE"
            elif cam_known and cam_confident:
                final_state = camera_voted_state
                decision_source = "CAM"
            elif v2i_known:
                final_state = v2i_state
                decision_source = "V2I"
            elif cam_known:
                final_state = camera_voted_state
                decision_source = "CAM_WEAK"
            else:
                final_state = "RED"
                decision_source = "FAILSAFE"

            target = state_to_target(final_state)

            if target != current_throttle:
                set_pwm(CH_THROTTLE, target)
                current_throttle = target

            state_changed = (
                final_state != stable_state or
                decision_source != last_source
            )

            if state_changed or (loop_count % PRINT_EVERY_N_LOOPS == 0):
                print(
                    f"[{loop_count:06d}] "
                    f"raw={raw_state:7s} cam={camera_voted_state:7s} "
                    f"v2i={v2i_state:7s} live={str(v2i_live):5s} final={final_state:7s} "
                    f"src={decision_source:8s} | rem={v2i_remaining:.2f}s "
                    f"| conf={result.confidence:.2f} "
                    f"| R={result.red_pixels:4d} Y={result.yellow_pixels:4d} G={result.green_pixels:4d} "
                    f"| throttle={'FWD' if target == ESC_FORWARD else 'STOP'} ({target})"
                )

            stable_state = final_state
            last_source = decision_source

            _log_writer.writerow([
                loop_count,
                f"{result.timestamp:.4f}",
                raw_state,
                camera_voted_state,
                v2i_state,
                int(v2i_live),
                f"{v2i_remaining:.2f}",
                final_state,
                decision_source,
                target,
                f"{result.confidence:.4f}",
                result.red_pixels,
                result.yellow_pixels,
                result.green_pixels,
            ])

            if loop_count % 10 == 0:
                _log_file.flush()

            annotated = annotate_frame(
                frame.copy(),
                detector,
                camera_voted_state,
                v2i_state,
                final_state,
                result,
                target,
                decision_source,
                v2i_remaining,
            )

            update_stream_frame(annotated)
            loop_count += 1

    except KeyboardInterrupt:
        print("[main] KeyboardInterrupt received.")
        safe_stop()
    except Exception as e:
        print(f"[ERROR] Unhandled exception: {e}")
        safe_stop()

    safe_stop()


if __name__ == '__main__':
    main()