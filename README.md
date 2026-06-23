# JPA Development Workflow

JPA (JetRacer Pro Autonomous) is the autonomous vehicle control system integrated into AIGST.

## Quick Daily Workflow

### 🚀 Start Working
```bash
# Open in AIGST (recommended - unified view)
cd ~/Desktop/AIGST
code AIGST.code-workspace
# Navigate to repos/JPA/ in file explorer

# OR work in isolation
cd ~/Desktop/JPA
code JPA.code-workspace
```

### 🔧 Make Changes
- **Production Code**: `0409/Dev/` folder
  - `3led_demoday.py` - Main control loop
  - `3led_detector.py` - Traffic light detection
  - `3led_v2i.py` - V2I communication
- **Debug Tools**: `Debug/` folder
- **Deployment**: `ssunc_perception/` folder

### ✅ Test Changes
```bash
# Local testing (if Jetson hardware available)
python3 0409/Dev/3led_demoday.py

# Syntax check
python3 -m py_compile 0409/Dev/*.py
```

### 💾 Commit & Push
```bash
git add .
git commit -m "feat: [brief description]"
git push
```

### 🚀 Deploy to Jetson
```bash
# From AIGST workspace
scp 0409/Dev/3led_demoday.py jetson@<JETSON_IP>:~/demoday.py
scp 0409/Dev/3led_detector.py jetson@<JETSON_IP>:~/jetson/ssunc_perception/traffic_light_detector.py
scp 0409/Dev/3led_v2i.py jetson@<JETSON_IP>:~/jetson/ssunc_perception/v2i_receiver.py

# Run on Jetson
ssh jetson@<JETSON_IP>
python3 -u ~/demoday.py
```

## Key Features

- **V2I Fusion**: Combines camera detection with ESP32 traffic light signals
- **RED/YELLOW/GREEN**: Full traffic light support with safety prioritization
- **MJPEG Streaming**: Real-time video feed on port 8080
- **CSV Logging**: Comprehensive decision tracking
- **Safe Shutdown**: Ctrl+C for graceful servo centering

## Architecture

```
Camera → HSV Detection → Majority Vote → V2I Fusion → Servo Control → Logging
    ↓           ↓             ↓           ↓           ↓          ↓
  BGR      RED/YELLOW/GREEN  7-frame     ESP32      PCA9685    CSV files
  frames    pixel counts     buffer     UDP:5005   PWM        timestamped
```

## Integration with AIGST

JPA is integrated as a folder in the unified AIGST workspace, giving you:
- **Unified View**: See JPA alongside AISkills/GSD in one workspace
- **Clear Ownership**: JPA files stay under `repos/JPA/` while sharing the AIGST Git history
- **Team Access**: Others can view JPA in AIGST context
- **Clean Boundaries**: No forced dependencies

When you update JPA, commit the scoped `repos/JPA/...` changes from the AIGST root.
