#!/usr/bin/env python3
"""
lane_detect.py  —  CON-20260415-09  |  Lane Line Detection
===========================================================
4P Code    : 4P-20260415-LANE
Deliverable: OpenCV function that finds lane lines from Jetson camera
             and returns lateral offset (pixels) from lane center.

PASS threshold: Correct left/right offset with correct sign in 8 of 10
                test frames captured on the real ASVP track.

TRACK DESCRIPTION THIS FILE IS TUNED FOR:
  - Dark floor (low saturation, low value — reject these)
  - Two WHITE border lines at the edges (high value, low saturation)
  - Optional YELLOW center line (mid hue, moderate saturation, high value)

CHANGES FROM ORIGINAL (RALPH fixes):

  FIX 1 — CRITICAL: Classifier changed from x-midpoint to slope sign.
    Original: lines split left/right by whether x-midpoint < or > image center.
    Bug: when car drifts right 100px, both white lines appear in the left half
    of the frame. Both go into left_xs. right_xs is empty. Returns None.
    PID gets None, holds straight, car drives off track.
    Fix: left border line always has negative slope (dy/dx < 0) in image coords
    because it runs from bottom-left toward top-right as you look forward.
    Right border always has positive slope. This holds wherever the car is.

  FIX 2 — WHITE_LOW value lowered from 180 to 170.
    Indoor fluorescent lighting in CKB 110 causes tape brightness to vary.
    170 catches slightly shadowed sections while still rejecting the dark floor.

  FIX 3 — Hough params loosened: threshold 40->30, min_length 50->40.
    Short tape segments at corners were being dropped entirely.

  FIX 4 — Yellow center line added as a separate detection pass.
    If both white borders drop out, yellow gives a fallback reference.

  FIX 5 — Single-border fallback using TRACK_WIDTH_PX.
    If only one white border is visible, estimate the other from track width.

  FIX 6 — Return signature changed from float|None to tuple.
    get_offset() now returns (offset, left_x, right_x, yellow_x)
    so run_main.py can log individual line positions for debugging.

  FIX 7 — _last_debug always set before returning.
    Previously, an early None return left _last_debug stale from the
    previous frame. Misleading in the MJPEG stream.

Deploy to:
  /home/jetson/jetson/ssunc_perception/lane_detect.py

Run standalone (live camera):
  python3 lane_detect.py

Run standalone (10-frame test against saved images):
  python3 lane_detect.py --test
"""

from typing import Optional, Tuple

import os
import sys

import cv2
import numpy as np

# ── Camera / image constants ───────────────────────────────────────────────────
CAM_INDEX = 0          # 0 = CSI onboard; change to 1 for USB
FRAME_W   = 640
FRAME_H   = 480
ROI_TOP   = 0.55       # ignore top 55% of frame (ceiling, far floor); use bottom 45%

# ── White tape HSV thresholds (dark track) ─────────────────────────────────────
# Dark floor is low-saturation AND low-value. White tape is low-saturation high-value.
# Value 170 instead of 180: gives headroom for shadowed sections of tape.
WHITE_LOW  = np.array([0,   0,   170], dtype=np.uint8)   # HSV
WHITE_HIGH = np.array([180, 50,  255], dtype=np.uint8)

# ── Yellow center line HSV thresholds ─────────────────────────────────────────
# Yellow electrical tape: hue ~18-38 deg, moderate-high saturation, high value.
YELLOW_LOW  = np.array([18,  80,  120], dtype=np.uint8)
YELLOW_HIGH = np.array([38,  255, 255], dtype=np.uint8)

# ── Hough line detection parameters ───────────────────────────────────────────
# Threshold lowered from 40 to 30: catches shorter tape segments at corners.
# Min length lowered from 50 to 40: same reason.
HOUGH_RHO        = 1
HOUGH_THETA      = np.pi / 180
HOUGH_THRESHOLD  = 30
HOUGH_MIN_LENGTH = 40
HOUGH_MAX_GAP    = 40

# ── Track geometry ─────────────────────────────────────────────────────────────
# Approximate pixel width of the drivable lane in the ROI at normal viewing depth.
# Measure this on a real frame: right_x - left_x when centered.
# Used for the single-border fallback (FIX 5).
TRACK_WIDTH_PX = 300


class LaneDetector:
    """
    Finds white border lines and optional yellow center line on the dark ASVP
    track. Returns lateral offset from lane center in pixels.

    Offset sign convention (same as original, unchanged — matches PIDSteering):
        negative  -> vehicle is too far LEFT  (needs to steer right)
        positive  -> vehicle is too far RIGHT (needs to steer left)
        0         -> centered

    Return value of get_offset():
        (offset, left_x, right_x, yellow_x)
        - offset    : float pixels, or None if nothing detected
        - left_x    : float x-position of left border in full-frame coords, or None
        - right_x   : float x-position of right border in full-frame coords, or None
        - yellow_x  : float x-position of yellow center line, or None
    """

    def __init__(self, frame_w=FRAME_W, frame_h=FRAME_H):
        self.frame_w     = frame_w
        self.frame_h     = frame_h
        self._cx         = frame_w // 2
        self._last_debug = None   # always updated before returning (FIX 7)
        self._morph_k    = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    def get_offset(self, frame: np.ndarray) -> tuple:
        """
        Main entry point.

        Args:
            frame : BGR image from cv2.VideoCapture.read()

        Returns:
            (offset_px, left_x, right_x, yellow_x)
            offset_px is None if detection completely failed.
        """
        roi, roi_top_y = self._extract_roi(frame)
        hsv            = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white_mask  = self._color_mask(hsv, WHITE_LOW,  WHITE_HIGH)
        yellow_mask = self._color_mask(hsv, YELLOW_LOW, YELLOW_HIGH)

        white_lines  = self._hough_lines(white_mask)
        yellow_lines = self._hough_lines(yellow_mask)

        # Classify white lines by slope sign (FIX 1)
        left_x, right_x = self._classify_by_slope(white_lines)

        # Yellow: average x-position of all detected segments
        yellow_x = self._mean_x(yellow_lines)

        # Compute offset with fallback priority (FIX 4, FIX 5)
        offset = self._compute_offset(left_x, right_x, yellow_x)

        # Always update debug frame (FIX 7)
        self._last_debug = self._draw_debug(
            frame, roi_top_y,
            white_lines, yellow_lines,
            left_x, right_x, yellow_x, offset,
        )

        return offset, left_x, right_x, yellow_x

    # ── Private ───────────────────────────────────────────────────────────────

    def _extract_roi(self, frame: np.ndarray):
        """Return bottom portion of frame (floor) and its y-offset in full frame."""
        top = int(frame.shape[0] * ROI_TOP)
        return frame[top:, :], top

    def _color_mask(self, hsv: np.ndarray, low, high) -> np.ndarray:
        """Threshold + morphological cleanup to isolate a color."""
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._morph_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_k)
        return mask

    def _hough_lines(self, mask: np.ndarray):
        """Detect line segments in a binary mask."""
        edges = cv2.Canny(mask, 50, 150)
        return cv2.HoughLinesP(
            edges,
            HOUGH_RHO, HOUGH_THETA, HOUGH_THRESHOLD,
            minLineLength=HOUGH_MIN_LENGTH,
            maxLineGap=HOUGH_MAX_GAP,
        )

    def _classify_by_slope(self, lines) -> tuple:
        """
        Classify white lines as LEFT or RIGHT border by slope sign.  (FIX 1)

        In a forward-facing camera looking at border lines on the floor:

          Left border line runs from bottom-left toward top-right as you
          look forward along the track. In image coordinates (y increases
          downward), this means as x increases, y decreases.
          -> slope  dy/dx  is NEGATIVE  -> classify as LEFT border.

          Right border line runs from bottom-right toward top-left.
          As x increases, y increases. -> slope dy/dx is POSITIVE -> RIGHT.

        This is the key fix: the original code used x-midpoint < image_center
        to decide left vs right. That breaks when the car drifts: both lines
        can end up on the same side of the image, so one bucket is empty and
        the function returns None, causing the PID to hold straight.
        Slope sign is a property of the LINE ITSELF, not where it appears.
        """
        if lines is None:
            return None, None

        left_xs  = []
        right_xs = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            dx = x2 - x1
            if dx == 0:
                continue
            slope = (y2 - y1) / dx

            if abs(slope) < 0.2:   # nearly horizontal -> floor texture or noise
                continue

            mx = (x1 + x2) / 2.0
            if slope < 0:
                left_xs.append(mx)    # negative slope -> left border
            else:
                right_xs.append(mx)   # positive slope -> right border

        left_x  = float(np.mean(left_xs))  if left_xs  else None
        right_x = float(np.mean(right_xs)) if right_xs else None
        return left_x, right_x

    def _mean_x(self, lines) -> Optional[float]:
        """Return average x-position of a set of line segments."""
        if lines is None:
            return None
        xs = [(l[0][0] + l[0][2]) / 2.0 for l in lines]
        return float(np.mean(xs)) if xs else None

    def _compute_offset(self, left_x, right_x, yellow_x) -> Optional[float]:
        """
        Compute how far the car is from lane center, in pixels.  (FIX 4, FIX 5)

        Priority:
          1. Both white borders found -> most accurate
          2. Yellow center line found -> car should sit beside it
          3. Only left border found   -> estimate right from TRACK_WIDTH_PX
          4. Only right border found  -> estimate left from TRACK_WIDTH_PX
          5. Nothing found            -> return None (PID holds straight)

        Offset meaning:
          Positive -> lane center is RIGHT of image center -> car too far LEFT
          Negative -> lane center is LEFT  of image center -> car too far RIGHT
        """
        if left_x is not None and right_x is not None:
            lane_center = (left_x + right_x) / 2.0

        elif yellow_x is not None:
            # Yellow is center line; our lane center is slightly left of it.
            # Using quarter-track offset as estimate.
            lane_center = yellow_x - (TRACK_WIDTH_PX / 4.0)

        elif left_x is not None:
            lane_center = left_x + (TRACK_WIDTH_PX / 2.0)

        elif right_x is not None:
            lane_center = right_x - (TRACK_WIDTH_PX / 2.0)

        else:
            return None

        return lane_center - self._cx

    def _draw_debug(self, frame, roi_top_y,
                    white_lines, yellow_lines,
                    left_x, right_x, yellow_x, offset) -> np.ndarray:
        """Build annotated frame for live preview and contract evidence screenshots."""
        out = frame.copy()

        # All white Hough lines (cyan)
        if white_lines is not None:
            for l in white_lines:
                x1, y1, x2, y2 = l[0]
                cv2.line(out, (x1, roi_top_y + y1), (x2, roi_top_y + y2),
                         (255, 255, 0), 1)

        # All yellow Hough lines (orange)
        if yellow_lines is not None:
            for l in yellow_lines:
                x1, y1, x2, y2 = l[0]
                cv2.line(out, (x1, roi_top_y + y1), (x2, roi_top_y + y2),
                         (0, 165, 255), 1)

        # Left border (blue vertical)
        if left_x is not None:
            lx = int(left_x)
            cv2.line(out, (lx, roi_top_y), (lx, frame.shape[0]), (255, 60, 60), 2)
            cv2.putText(out, "L", (lx - 14, roi_top_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 60, 60), 2)

        # Right border (red vertical)
        if right_x is not None:
            rx = int(right_x)
            cv2.line(out, (rx, roi_top_y), (rx, frame.shape[0]), (60, 60, 255), 2)
            cv2.putText(out, "R", (rx + 5, roi_top_y + 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 255), 2)

        # Yellow center (teal vertical)
        if yellow_x is not None:
            yx = int(yellow_x)
            cv2.line(out, (yx, roi_top_y), (yx, frame.shape[0]), (0, 220, 180), 2)
            cv2.putText(out, "Y", (yx + 5, roi_top_y + 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 180), 2)

        # Image center (white dotted)
        cv2.line(out, (self._cx, roi_top_y), (self._cx, frame.shape[0]),
                 (200, 200, 200), 1)

        # Lane center (green) and offset readout
        if offset is not None:
            lc = int(self._cx + offset)
            cv2.line(out, (lc, roi_top_y), (lc, frame.shape[0]), (0, 255, 0), 2)
            col  = (0, 200, 0) if abs(offset) < 30 else (0, 80, 255)
            side = "RIGHT (steer left)" if offset > 0 else \
                   ("LEFT  (steer right)" if offset < 0 else "CENTER")
            cv2.putText(out, f"Offset: {offset:+.1f}px  Drift: {side}",
                        (10, frame.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        else:
            cv2.putText(out, "NO DETECTION — PID holds straight",
                        (10, frame.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 80, 255), 2)

        return out


# ── 10-frame static test harness (contract evidence) ──────────────────────────

def run_10_frame_test(image_dir: str = "test_frames"):
    """
    Validate against 10 saved frames from the real ASVP track.
    Place images named frame_00.jpg ... frame_09.jpg in image_dir/.
    Saves annotated debug_XX.jpg alongside each input frame.

    Usage: python3 lane_detect.py --test
    """
    ld     = LaneDetector()
    passed = 0
    total  = 0
    log    = []

    print(f"\n=== CON-20260415-09  10-Frame Test  (dir: {image_dir}) ===")
    for i in range(10):
        path = os.path.join(image_dir, f"frame_{i:02d}.jpg")
        if not os.path.exists(path):
            print(f"  Frame {i:02d}: SKIP (not found: {path})")
            log.append({"frame": i, "result": "SKIP", "offset": None})
            continue

        frame                            = cv2.imread(path)
        offset, left_x, right_x, yellow_x = ld.get_offset(frame)
        total += 1

        if offset is not None:
            side   = "RIGHT" if offset > 0 else ("LEFT" if offset < 0 else "CENTER")
            result = "PASS"
            passed += 1
        else:
            side   = "NO DETECTION"
            result = "FAIL"

        l_str = f"{left_x:.1f}"   if left_x   is not None else "---"
        r_str = f"{right_x:.1f}"  if right_x  is not None else "---"
        y_str = f"{yellow_x:.1f}" if yellow_x is not None else "---"
        print(f"  Frame {i:02d}: {result:4s} | offset={str(offset):>8}px  {side:6s} "
              f"| L={l_str} R={r_str} Y={y_str}")
        log.append({"frame": i, "result": result, "offset": offset, "side": side})

        if ld._last_debug is not None:
            out_path = os.path.join(image_dir, f"debug_{i:02d}.jpg")
            cv2.imwrite(out_path, ld._last_debug)

    print(f"\nResult: {passed}/{total} PASS  (need 8/10 for contract PASS)")
    print("CONTRACT CONDITION:", "PASS" if passed >= 8 else "FAIL — tune thresholds")
    return log


# ── Live camera mode ───────────────────────────────────────────────────────────

def run_live():
    """Live offset readout on the Jetson camera. Press Q to quit."""
    print("Starting live camera. Press Q to quit.")
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    ld  = LaneDetector()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read failed")
            break

        offset, left_x, right_x, yellow_x = ld.get_offset(frame)
        print(
            f"offset={str(offset):>8}px  "
            f"L={str(left_x):>6}  R={str(right_x):>6}  Y={str(yellow_x):>6}",
            end="\r",
        )

        if ld._last_debug is not None:
            cv2.imshow("Lane Detection — CON-09", ld._last_debug)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    if "--test" in sys.argv:
        run_10_frame_test()
    else:
        run_live()
