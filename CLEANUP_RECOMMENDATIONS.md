# JPA Repository Cleanup Recommendations

## Summary
The repository contains multiple iterations of the same core system. The **most current production code** is in `0409/Dev/` with the `3led_*` prefix files. All other versions are outdated and should be consolidated or deleted.

---

## 0409/Dev/ (KEEP - These are Production)

### ✅ 3led_demoday.py
**Status:** KEEP - Main orchestration script  
**Features:**
- Complete camera + perception + V2I fusion logic
- Full control loop with PCA9685 servo/throttle control
- MJPEG streaming server on port 8080
- Comprehensive CSV logging
- Support for RED / YELLOW / GREEN signals
- Majority voting over 7-frame buffer
- Safe fail-safe logic (defaults to RED on uncertainty)
- Proper signal handling for clean shutdown

**Purpose:** This is the main entry point that ties everything together.

### ✅ 3led_detector.py
**Status:** KEEP - Detection module  
**Features:**
- `DeterministicTrafficLightDetector` class
- HSV-based color detection with safety-priority tie-breaking
- Supports RED / YELLOW / GREEN / UNKNOWN
- Returns pixel counts and confidence scores
- Configurable ROI, erosion, and thresholds

**Purpose:** Imported by 3led_demoday.py to detect traffic light signals.

### ✅ 3led_v2i.py
**Status:** KEEP - V2I communication module  
**Features:**
- `V2IReceiver` class
- UDP listener on port 5005
- Parses JSON from ESP32 traffic light controller
- Thread-safe with 2-second timeout fallback
- Tracks packet count and signal age

**Purpose:** Imported by 3led_demoday.py to receive vehicle-to-infrastructure signals.

---

## 0409/ (DELETE - Outdated Intermediate Versions)

### ❌ demoday.py
**Reason:** Outdated predecessor to 3led_demoday.py  
**Differences:**
- Missing V2I support entirely
- Only supports RED/GREEN (no YELLOW)
- Less robust logging
- No mention of voting buffer in visible code

### ❌ traffic_light_detector.py
**Reason:** Outdated predecessor to 3led_detector.py  
**Differences:**
- Only supports RED/GREEN (no YELLOW detection)
- Uses old Signal enum without YELLOW
- DetectionResult dataclass missing yellow_pixels field
- Less sophisticated color masking

### ❌ v2i_receiver.py
**Reason:** Outdated predecessor to 3led_v2i.py  
**Differences:**
- Only supports RED/GREEN in VALID_STATES (no YELLOW)
- Similar functionality but older implementation

---

## Root Level (DELETE - Old Experimental/Debug Versions)

### ❌ demodaystream.py
**Reason:** Old version with streaming but no V2I  
**Status:** Superseded by 3led_demoday.py which has better streaming + fusion logic

### ❌ run.py
**Reason:** Minimal/incomplete implementation  
**Status:** Missing proper voting, V2I support, and clean shutdown logic

### ❌ running.py
**Reason:** Similar to run.py with slightly different constants  
**Status:** Obsolete variant that was likely used for testing only

### ❌ running_trim.py
**Reason:** Calibration/trimming script  
**Status:** Keep only if active calibration is needed; consider moving to Debug/ or archive

### ❌ ground_test.py
**Reason:** Test script with unclear purpose  
**Status:** Delete unless it serves a specific ongoing test function

---

## Debug/ & Other Folders

**Recommended Action:**
- **Debug/** folder: Keep for active development/troubleshooting, but clean up old versions
  - `steer_trim.py`, `steer_trim_s.py` - Keep only one canonical trim script
  - Delete duplicate trim files
- **files/** folder: Contains SSH config and old run_main.py variants - likely obsolete
- **ssunc_perception/traffic_light_detector.py**: Check if this is a copy of old detector; if so, it should pull from 0409/Dev/3led_detector.py instead

---

## Recommended Cleanup Actions

### Phase 1: Safe Removal (No Loss of Functionality)
```
DELETE:
  /0409/demoday.py
  /0409/traffic_light_detector.py
  /0409/v2i_receiver.py
  /demodaystream.py
  /run.py
  /running.py
  /ground_test.py
```

### Phase 2: Consolidate (With Review)
```
REVIEW:
  /running_trim.py - Keep if active calibration tool, else archive
  /Debug/ - Consolidate duplicate trim scripts
  /files/run_main*.py - Delete old variants
```

### Phase 3: Path Updates
Update `3led_demoday.py` imports to match actual deployment paths on Jetson:
```python
sys.path.insert(0, '/home/jetson/jetson/ssunc_perception')
from traffic_light_detector import DeterministicTrafficLightDetector  # <- source
from v2i_receiver import V2IReceiver  # <- source
```

Ensure on Jetson that:
- `/home/jetson/jetson/ssunc_perception/traffic_light_detector.py` = Dev version
- `/home/jetson/jetson/ssunc_perception/v2i_receiver.py` = Dev version (or deploy directly from /0409/Dev/)

---

## Updated Repository Structure (After Cleanup)

```
JPA/
├── CLAUDE.md
├── JPA.code-workspace
├── CLEANUP_RECOMMENDATIONS.md
│
├── 0409/
│   └── Dev/
│       ├── 3led_demoday.py          (MAIN SCRIPT - deploy to /home/jetson/)
│       ├── 3led_detector.py         (deploy to /home/jetson/jetson/ssunc_perception/)
│       └── 3led_v2i.py              (deploy to /home/jetson/jetson/ssunc_perception/)
│
├── Debug/
│   ├── steer_trim.py                (canonical trim tool)
│   ├── minimal.py
│   └── [other active debug scripts]
│
├── logs/                             (runtime logs)
│   └── run_*.csv
│
└── [ARCHIVE or DELETE non-essential folders]
    ├── Reports/
    └── files/
```

---

## Key Takeaway

**Keep the 3 files in 0409/Dev/ only.**  
They represent the latest, most complete implementation with:
- ✅ V2I vehicle-to-infrastructure fusion
- ✅ All three traffic light colors (RED/YELLOW/GREEN)
- ✅ Robust voting and fail-safe logic
- ✅ Streaming and comprehensive logging

Everything else is obsolete and creates confusion and maintenance debt.
