#!/usr/bin/env python3
"""
v2i_receiver.py — ACDC Phase 2 Jetson V2I UDP Receiver

Listens for UDP packets from the ESP32 traffic light controller on port 5005.

Expected payload (any of these key names are accepted for time remaining):
  {"state":"GREEN","seconds_remaining":6.4,"mode":"AUTO","timestamp_ms":12345}
  {"state":"GREEN","time_remaining":6.4,...}
  {"state":"GREEN","remaining":6.4,...}
  {"state":"GREEN","timeRemaining":6.4,...}

The first received packet's keys are printed so the exact format can be confirmed.

Valid states:
  RED, YELLOW, GREEN, UNKNOWN
"""

import json
import socket
import threading
import time


class V2IReceiver:
    UDP_PORT = 5005
    TIMEOUT_S = 2.0
    BIND_IP = "0.0.0.0"

    VALID_STATES = {"RED", "YELLOW", "GREEN", "UNKNOWN"}

    # Key names tried in order for the time-remaining field.
    # First match wins. Extend if the ESP32 uses a different name.
    SECONDS_REM_KEYS = (
        "seconds_remaining",
        "time_remaining",
        "remaining",
        "timeRemaining",
        "secondsRemaining",
        "seconds_rem",
    )

    def __init__(self):
        self._lock = threading.Lock()
        self._state = "UNKNOWN"
        self._seconds_rem = 0.0
        self._mode = "UNKNOWN"
        self._last_rx_time = 0.0
        self._packet_count = 0
        self._thread = None
        self._running = False
        self._fallback_logged = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        print(f"[V2I] Receiver started on UDP port {self.UDP_PORT}")

    def stop(self):
        self._running = False

    def get_latest(self):
        now = time.time()
        with self._lock:
            age = now - self._last_rx_time if self._last_rx_time > 0 else 999.0
            if age > self.TIMEOUT_S:
                if not self._fallback_logged:
                    print(f"[V2I] No packet for {age:.1f}s — returning UNKNOWN")
                    self._fallback_logged = True
                return "UNKNOWN", 0.0

            self._fallback_logged = False
            return self._state, max(0.0, self._seconds_rem)

    @property
    def packet_count(self):
        with self._lock:
            return self._packet_count

    @property
    def is_live(self):
        with self._lock:
            age = time.time() - self._last_rx_time if self._last_rx_time > 0 else 999.0
            return age <= self.TIMEOUT_S

    def _listen_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.BIND_IP, self.UDP_PORT))
        sock.settimeout(1.0)

        print(f"[V2I] Socket bound to {self.BIND_IP}:{self.UDP_PORT}")

        while self._running:
            try:
                data, addr = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[V2I] Socket error: {e}")
                time.sleep(0.1)
                continue

            try:
                payload = json.loads(data.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"[V2I] Bad packet from {addr}: {e}")
                continue

            state = str(payload.get("state", "UNKNOWN")).upper()
            if state not in self.VALID_STATES:
                state = "UNKNOWN"

            # Try each known key name for time remaining; first match wins.
            seconds_rem = 0.0
            matched_key = None
            for key in self.SECONDS_REM_KEYS:
                if key in payload:
                    try:
                        seconds_rem = float(payload[key])
                        matched_key = key
                    except Exception:
                        seconds_rem = 0.0
                    break

            mode = str(payload.get("mode", "UNKNOWN"))

            with self._lock:
                self._state = state
                self._seconds_rem = seconds_rem
                self._mode = mode
                self._last_rx_time = time.time()
                self._packet_count += 1
                packet_count = self._packet_count

            if packet_count == 1:
                # Dump raw keys so we can confirm the exact ESP32 payload format.
                print(
                    f"[V2I] First packet from {addr[0]}  keys={list(payload.keys())}  "
                    f"state={state}  rem_key={matched_key!r}  rem={seconds_rem:.2f}s"
                )
            elif packet_count % 150 == 0:
                print(
                    f"[V2I] #{packet_count:05d} from {addr[0]}  "
                    f"state={state}  rem={seconds_rem:.2f}s  mode={mode}"
                )

        sock.close()
        print("[V2I] Receiver stopped.")


if __name__ == "__main__":
    print("[test] V2I standalone test — Ctrl+C to stop")
    print(f"[test] Waiting for ESP32 UDP on port {V2IReceiver.UDP_PORT}...")

    v2i = V2IReceiver()
    v2i.start()

    try:
        while True:
            state, rem = v2i.get_latest()
            live = "LIVE" if v2i.is_live else "FALLBACK"
            print(f"[test] [{live}] state={state:7s} rem={rem:.2f}s packets={v2i.packet_count}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[test] Stopped.")
        v2i.stop()
