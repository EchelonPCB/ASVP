#!/usr/bin/env python3
"""
traffic_light_detector.py — ACDC JetRacer Pro
Deterministic HSV-based traffic light perception pipeline.

Now supports:
  RED / YELLOW / GREEN / UNKNOWN

Deploy to:
  /home/jetson/jetson/ssunc_perception/traffic_light_detector.py
"""

import time
from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np


class Signal(Enum):
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    UNKNOWN = "UNKNOWN"


@dataclass
class DetectionResult:
    signal: str
    confidence: float
    red_pixels: int
    yellow_pixels: int
    green_pixels: int
    timestamp: float


class DeterministicTrafficLightDetector:
    """
    ROI crop → BGR→HSV → color masks → erosion → classify dominant signal.

    Safety doctrine:
      RED wins ties over YELLOW/GREEN
      YELLOW wins ties over GREEN
      UNKNOWN if confidence / pixel threshold not met

    Current intended semantics:
      RED    = STOP
      YELLOW = STOP  (until timing+distance arbitration exists)
      GREEN  = GO
    """

    def __init__(
        self,
        roi_top_left=(240, 100),
        roi_bottom_right=(400, 300),
        min_pixel_threshold=50,
        confidence_threshold=0.60,
        erosion_iterations=1,
    ):
        self.roi_top_left = roi_top_left
        self.roi_bottom_right = roi_bottom_right
        self.min_pixel_threshold = min_pixel_threshold
        self.confidence_threshold = confidence_threshold
        self.erosion_iterations = erosion_iterations

        self._kernel = np.ones((3, 3), np.uint8)

        # RED wraps hue
        self._red_lower1 = np.array([0, 120, 70])
        self._red_upper1 = np.array([10, 255, 255])
        self._red_lower2 = np.array([170, 120, 70])
        self._red_upper2 = np.array([180, 255, 255])

        # YELLOW
        self._yellow_lower = np.array([18, 120, 120])
        self._yellow_upper = np.array([38, 255, 255])

        # GREEN
        self._green_lower = np.array([40, 150, 100])
        self._green_upper = np.array([90, 255, 255])

    def detect(self, frame_bgr: np.ndarray) -> DetectionResult:
        ts = time.time()

        roi = self._crop_roi(frame_bgr)
        if roi is None:
            return DetectionResult("UNKNOWN", 0.0, 0, 0, 0, ts)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask = self._red_mask(hsv)
        yellow_mask = cv2.inRange(hsv, self._yellow_lower, self._yellow_upper)
        green_mask = cv2.inRange(hsv, self._green_lower, self._green_upper)

        if self.erosion_iterations > 0:
            red_mask = cv2.erode(red_mask, self._kernel, iterations=self.erosion_iterations)
            yellow_mask = cv2.erode(yellow_mask, self._kernel, iterations=self.erosion_iterations)
            green_mask = cv2.erode(green_mask, self._kernel, iterations=self.erosion_iterations)

        red_pixels = int(cv2.countNonZero(red_mask))
        yellow_pixels = int(cv2.countNonZero(yellow_mask))
        green_pixels = int(cv2.countNonZero(green_mask))

        return self._classify(red_pixels, yellow_pixels, green_pixels, ts)

    def _crop_roi(self, frame_bgr: np.ndarray):
        x1, y1 = self.roi_top_left
        x2, y2 = self.roi_bottom_right
        h, w = frame_bgr.shape[:2]

        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h))

        if x2 <= x1 or y2 <= y1:
            return None

        return frame_bgr[y1:y2, x1:x2]

    def _red_mask(self, hsv: np.ndarray) -> np.ndarray:
        m1 = cv2.inRange(hsv, self._red_lower1, self._red_upper1)
        m2 = cv2.inRange(hsv, self._red_lower2, self._red_upper2)
        return cv2.bitwise_or(m1, m2)

    def _classify(self, red_pixels: int, yellow_pixels: int, green_pixels: int, ts: float) -> DetectionResult:
        total = red_pixels + yellow_pixels + green_pixels

        if total < self.min_pixel_threshold:
            return DetectionResult("UNKNOWN", 0.0, red_pixels, yellow_pixels, green_pixels, ts)

        counts = {
            "RED": red_pixels,
            "YELLOW": yellow_pixels,
            "GREEN": green_pixels,
        }

        # Safe-priority tie break:
        # RED > YELLOW > GREEN
        ordered = sorted(
            counts.items(),
            key=lambda kv: (kv[1], {"RED": 3, "YELLOW": 2, "GREEN": 1}[kv[0]]),
            reverse=True
        )

        dominant_signal, dominant_count = ordered[0]
        confidence = dominant_count / max(1, total)

        if confidence < self.confidence_threshold:
            return DetectionResult("UNKNOWN", float(confidence), red_pixels, yellow_pixels, green_pixels, ts)

        return DetectionResult(dominant_signal, float(confidence), red_pixels, yellow_pixels, green_pixels, ts)
