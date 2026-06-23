#!/usr/bin/env python3
"""
lane_detect_track.py  —  JPA tracktest lane detection
======================================================
Track-specific variant of lane_detect.py for the ASVP/SSUNC single-lane course.

LANE GEOMETRY (measured from track photos, 2026-04-23):
  LEFT boundary  : YELLOW electrical tape — the center divider between two lanes
  RIGHT boundary : WHITE electrical tape  — the outer edge of the right lane
  Car position   : Between yellow (left) and white (right)

DIFFERENCES FROM lane_detect.py (0423/ssunc_perception):
  1. Detection strategy changed from two-white-border to yellow-left + white-right.
     Yellow mask → left boundary position.
     White mask  → right boundary position.
  2. _classify_by_slope() still used for WHITE lines only (right border detection).
     A single positive-slope white line at the right edge is the expected output.
  3. Lane center = (yellow_x + white_x) / 2  when both are found.
  4. Single-border fallbacks still active (TRACK_WIDTH_PX).
  5. Debug overlay updated: yellow vertical = left bound, white vertical = right bound.

OFFSET SIGN CONVENTION (same as lane_detect.py — compatible with PIDSteering):
  Positive → car is RIGHT of lane center → steer left (PID increases ticks)
  Negative → car is LEFT  of lane center → steer right (PID decreases ticks)
  None     → no boundaries detected → PID holds straight

CALIBRATION NOTES:
  - TRACK_WIDTH_PX: measure right_x − yellow_x on a centered test frame
  - ROI_TOP: set so the ROI covers the floor ahead, not the ceiling or far walls
  - Adjust HSV thresholds if lighting conditions at CKB 110 change

Deploy to:
  /home/jetson/jetson/tracktest/perception/lane_detect_track.py

Standalone live test:
  python3 lane_detect_track.py
"""

from typing import Optional, Tuple

import os
import sys

import cv2
import numpy as np

# ── Camera / image constants ───────────────────────────────────────────────────
CAM_INDEX = 0          # 0 = CSI onboard; 1 for USB
FRAME_W   = 640
FRAME_H   = 480
ROI_TOP   = 0.55       # ignore top 55% — ceiling / far floor

# ── White tape HSV (right boundary) ───────────────────────────────────────────
WHITE_LOW  = np.array([0,   0,   170], dtype=np.uint8)
WHITE_HIGH = np.array([180, 50,  255], dtype=np.uint8)

# ── Yellow tape HSV (left boundary — center divider) ──────────────────────────
YELLOW_LOW  = np.array([18,  80,  120], dtype=np.uint8)
YELLOW_HIGH = np.array([38,  255, 255], dtype=np.uint8)

# ── Hough parameters ───────────────────────────────────────────────────────────
HOUGH_RHO        = 1
HOUGH_THETA      = np.pi / 180
HOUGH_THRESHOLD  = 30
HOUGH_MIN_LENGTH = 40
HOUGH_MAX_GAP    = 40

# ── Track geometry ─────────────────────────────────────────────────────────────
# Approximate pixel distance between yellow left border and white right border
# when the car is at the center of the lane and the camera sees the ROI floor.
# Measure: run live mode, note right_x − yellow_x when car is centered.
TRACK_WIDTH_PX = 280


class LaneDetectorTrack:
    """
    Detects yellow (left) and white (right) lane boundaries and returns
    the lateral offset of the car from lane center.

    Return value of get_offset():
        (offset_px, yellow_x, white_x)
        - offset_px : signed float pixels, or None if detection failed
        - yellow_x  : x-position of yellow (left) boundary, or None
        - white_x   : x-position of white  (right) boundary, or None
    """

    def __init__(self, frame_w: int = FRAME_W, frame_h: int = FRAME_H):
        self.frame_w     = frame_w
        self.frame_h     = frame_h
        self._cx         = frame_w // 2
        self._last_debug = None
        self._morph_k    = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    # ── Public ────────────────────────────────────────────────────────────────

    def get_offset(self, frame: np.ndarray) -> tuple:
        """
        Main entry point.

        Args:
            frame : BGR image from cv2.VideoCapture.read()

        Returns:
            (offset_px, yellow_x, white_x)
            offset_px is None if neither boundary is detected.
        """
        roi, roi_top_y = self._extract_roi(frame)
        hsv            = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white_mask  = self._color_mask(hsv, WHITE_LOW,  WHITE_HIGH)
        yellow_mask = self._color_mask(hsv, YELLOW_LOW, YELLOW_HIGH)

        white_lines  = self._hough_lines(white_mask)
        yellow_lines = self._hough_lines(yellow_mask)

        # Yellow lines → left boundary (use mean x of all yellow segments)
        yellow_x = self._mean_x(yellow_lines)

        # White lines → right boundary
        # Use slope-sign classifier: right border has positive slope.
        # We take only the right-classified bucket from white lines.
        white_x = self._classify_right_border(white_lines)

        offset = self._compute_offset(yellow_x, white_x)

        self._last_debug = self._draw_debug(
            frame, roi_top_y,
            white_lines, yellow_lines,
            yellow_x, white_x, offset,
        )

        return offset, yellow_x, white_x

    # ── Private ───────────────────────────────────────────────────────────────

    def _extract_roi(self, frame: np.ndarray) -> tuple:
        top = int(frame.shape[0] * ROI_TOP)
        return frame[top:, :], top

    def _color_mask(self, hsv: np.ndarray, low, high) -> np.ndarray:
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._morph_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_k)
        return mask

    def _hough_lines(self, mask: np.ndarray):
        edges = cv2.Canny(mask, 50, 150)
        return cv2.HoughLinesP(
            edges,
            HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD,
            minLineLength=HOUGH_MIN_LENGTH,
            maxLineGap=HOUGH_MAX_GAP,
        )

    def _classify_right_border(self, lines) -> Optional[float]:
        """
        From white Hough lines, extract only those with positive slope
        (right border of the lane) and return their average x-midpoint.

        Positive slope in image coords: as x increases, y increases.
        The right border runs from bottom-right toward top-left → positive slope.
        """
        if lines is None:
            return None

        right_xs = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            if dx == 0:
                continue
            slope = (y2 - y1) / dx
            if abs(slope) < 0.2:   # nearly horizontal → noise
                continue
            if slope > 0:
                right_xs.append((x1 + x2) / 2.0)

        return float(np.mean(right_xs)) if right_xs else None

    def _mean_x(self, lines) -> Optional[float]:
        """Average x-midpoint of a set of Hough line segments."""
        if lines is None:
            return None
        xs = [(l[0][0] + l[0][2]) / 2.0 for l in lines]
        return float(np.mean(xs)) if xs else None

    def _compute_offset(self, yellow_x: Optional[float],
                        white_x: Optional[float]) -> Optional[float]:
        """
        Compute signed pixel offset from lane center.

        Priority:
          1. Both boundaries found → lane_center = (yellow_x + white_x) / 2
          2. Only yellow found    → estimate white from TRACK_WIDTH_PX
          3. Only white found     → estimate yellow from TRACK_WIDTH_PX
          4. Nothing found        → return None (PID holds straight)

        Positive offset: car is RIGHT of lane center → PID steers left.
        Negative offset: car is LEFT  of lane center → PID steers right.
        """
        if yellow_x is not None and white_x is not None:
            lane_center = (yellow_x + white_x) / 2.0

        elif yellow_x is not None:
            # Only left boundary visible; estimate right
            lane_center = yellow_x + (TRACK_WIDTH_PX / 2.0)

        elif white_x is not None:
            # Only right boundary visible; estimate left
            lane_center = white_x - (TRACK_WIDTH_PX / 2.0)

        else:
            return None

        return lane_center - self._cx

    def _draw_debug(self, frame, roi_top_y,
                    white_lines, yellow_lines,
                    yellow_x, white_x, offset) -> np.ndarray:
        out = frame.copy()

        # Raw white Hough lines (cyan tint)
        if white_lines is not None:
            for l in white_lines:
                x1, y1, x2, y2 = l[0]
                cv2.line(out,
                         (x1, roi_top_y + y1), (x2, roi_top_y + y2),
                         (180, 255, 180), 1)

        # Raw yellow Hough lines (orange tint)
        if yellow_lines is not None:
            for l in yellow_lines:
                x1, y1, x2, y2 = l[0]
                cv2.line(out,
                         (x1, roi_top_y + y1), (x2, roi_top_y + y2),
                         (0, 165, 255), 1)

        # Yellow boundary (left) — teal vertical
        if yellow_x is not None:
            yx = int(yellow_x)
            cv2.line(out, (yx, roi_top_y), (yx, frame.shape[0]), (0, 220, 180), 2)
            cv2.putText(out, "Y(L)", (yx + 4, roi_top_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 220, 180), 2)

        # White boundary (right) — blue vertical
        if white_x is not None:
            wx = int(white_x)
            cv2.line(out, (wx, roi_top_y), (wx, frame.shape[0]), (255, 80, 80), 2)
            cv2.putText(out, "W(R)", (wx + 4, roi_top_y + 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 80, 80), 2)

        # Image center (grey)
        cv2.line(out, (self._cx, roi_top_y), (self._cx, frame.shape[0]),
                 (200, 200, 200), 1)

        # Lane center (green) + offset readout
        if offset is not None:
            lc  = int(self._cx + offset)
            cv2.line(out, (lc, roi_top_y), (lc, frame.shape[0]), (0, 255, 0), 2)
            col  = (0, 200, 0) if abs(offset) < 30 else (0, 80, 255)
            side = ("RIGHT→steer L" if offset > 0 else
                    ("LEFT→steer R" if offset < 0 else "CENTER"))
            cv2.putText(out, f"offset={offset:+.1f}px  {side}",
                        (10, frame.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, col, 2)
        else:
            cv2.putText(out, "NO DETECTION — PID holds center",
                        (10, frame.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 80, 255), 2)

        return out


# ── Live camera mode ───────────────────────────────────────────────────────────

def _gstreamer_pipeline():
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def run_live():
    """Live offset readout on the Jetson camera. Press Q to quit."""
    print("LaneDetectorTrack — live mode. Press Q to quit.")
    print("Expected: yellow line = left boundary, white line = right boundary")

    cap = cv2.VideoCapture(_gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("[ERROR] Camera failed to open. Trying /dev/video0...")
        cap = cv2.VideoCapture(0)

    ld = LaneDetectorTrack()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        offset, yellow_x, white_x = ld.get_offset(frame)
        y_str = f"{yellow_x:.1f}" if yellow_x is not None else "---"
        w_str = f"{white_x:.1f}"  if white_x  is not None else "---"
        print(
            f"offset={str(offset):>8}px  yellow(L)={y_str:>7}  white(R)={w_str:>7}",
            end="\r",
        )

        if ld._last_debug is not None:
            cv2.imshow("LaneDetectorTrack", ld._last_debug)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_live()
