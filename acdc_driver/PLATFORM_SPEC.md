# ACDC JetRacer Fleet — Full Spec Sheet (Pro vs Reg)

The fleet is **two different JetRacer platforms** that drive the wheels through
**completely different hardware.** Anything that hardcodes one silently fails on
the other (this is what bit the motor bring-up). This is the authoritative
distinction — keep it in sync with `car_config.py`.

> **Naming:** Waveshare's **"JetRacer AI Kit"** = the regular **Reg** car here;
> **"JetRacer Pro AI Kit"** = **Pro**. Older repo docs say "AI Kit (non-Pro)" —
> same car as **Reg**. Legend: ✅ confirmed · ❓ verify.

---

## Actuation map — every car has TWO systems: a steering actuator + a drive motor

| System | JetRacer **Pro** | JetRacer **Reg** |
|---|---|---|
| **Steering actuator** (servo) | PCA9685 `0x40` · ch0 · reg `0x06` — center 320, L370/R280 | PCA9685 `0x40` · ch0 · reg `0x06` — center 425, L300/R500 (inverted) |
| **Drive motor** | ESC pulse · PCA9685 `0x40` · ch1 · reg `0x0A` — 307/330/345 | TB6612 H-bridge · PCA9685 `0x60` — R: pwm ch0 (`0x06`) / dir ch1,2 · L: pwm ch6 (`0x1E`) / dir ch7,8 |

- **Pro** runs both systems off **one** PCA9685 (`0x40`): steering = ch0, motor = ch1. The motor never leaves `0x40`.
- **Reg** splits them: steering actuator on `0x40`, drive motor on a **second** PCA9685 `0x60`.
- Register `0x06` appears for steering (on `0x40`) **and** for Reg's right-motor PWM (on `0x60`) — same channel number, different chip. Always read a register **with** its chip address; that's where the actuator/motor swap happens.

---

## A. Universal — identical on both (lives in the image + driver, written once)

| Layer | Spec |
|---|---|
| Compute | Jetson Nano Dev Kit, **4 GB (B01)** |
| OS / SDK | Ubuntu **18.04.6** · JetPack **4.x** (4.5.1 / R32.5.2 verified on the Reg car) |
| CUDA / Python (host) | CUDA **10.2** · Python **3.6** |
| I2C | **bus 1** (bus 0 dead on both) |
| **Steering actuator** | PCA9685 **@ 0x40 · channel 0 · register `LED0_ON_L = 0x06`** (4-byte block `0x06–0x09`). **Identical on Pro & Reg** — only the calibration ticks differ. Source: `demoday.py` (0409 & 0423). |
| PCA9685 register map | channel *n* PWM block = **`0x06 + 4·n`** → steer (ch0) = `0x06`, throttle (ch1, Pro) = `0x0A`. Init: prescale **121 → 50 Hz**, raw `smbus2`. |
| ROS layer | **ROS 2 Foxy** in Docker (Ubuntu 20.04 / Python 3.8) · `ROS_DOMAIN_ID=42` |
| Driver | `acdc_driver_node.py` — `/cmd_vel` → car, 0.5 s watchdog failsafe |
| Camera | CSI (IMX219) via `nvarguscamerasrc` — **native only**, no nvargus in the Foxy container |

**Software stack (both platforms):**
- **Host (native, no Docker):** the SSUNC/ASVP perception — `demoday.py`, `lane_detect.py`, `traffic_light_detector.py`, `v2i_receiver.py`. Python 3.6 + OpenCV + CSI camera. *This is the flagship that won AI Innovation Day.*
- **Container (Docker):** ROS 2 Foxy — the `/cmd_vel` driver + Nav2/SLAM + multi-car DDS coordination.

---

## B. Side-by-side

| Parameter | JetRacer **Pro** (6 cars · 0409/0423) | JetRacer **Reg** (today's car) |
|---|---|---|
| Chassis | JetRacer Pro AI Kit | JetRacer AI Kit (regular) |
| Steering actuator | **PCA9685 0x40 · ch0 · reg `0x06`** | **same** — 0x40 · ch0 · reg `0x06` (inverted ticks) |
| Drive hardware | **ESC** (brushless-style) | **2× 37-520 DC encoder gearmotors** |
| Drive controller | PCA9685 **0x40, CH1** (reg `0x0A`, same chip as steering) | **2nd PCA9685 @ 0x60** + TB6612 H-bridge |
| Throttle command | ESC pulse — neutral **307**, fwd **330** (rush **345**) | PWM + direction pins, per motor |
| → right motor (A) | — | **pwm 0, dir 1/2** ✅ |
| → left motor (B) | — | **pwm 6, dir 7/8** ✅ (not the standard 5/3,4) |
| Steering calibration | center **320**, L **370** / R **280** (higher = left) | center **425**, L **300** / R **500** (**inverted**) |
| `drive` (config key) | `esc` | `dc_hbridge` |
| Power | onboard battery pack (Waveshare Pro board) ❓verify | power brick + **UPS w/ 3× 18650**; motor rail from battery |
| Encoders | n/a | present on gearmotors (wiring/use ❓ TBD) |
| LiDAR | none — RPLIDAR is future (Dr. Lee's SLAM guide) ❓ | none — same |
| Drive validation | ✅ fully validated | ✅ steering + both motors confirmed |

---

## C. Per-platform detail

### JetRacer Pro — I2C bus 1
| Addr | Device | Use |
|---|---|---|
| 0x40 | PCA9685 | CH0 steering servo (reg `0x06`) · CH1 ESC throttle (reg `0x0A`) |
| 0x70 | (alias) | PCA9685 all-call (appears once initialized) |

Calibration: `steer_center 320`, `steer_left 370`, `steer_right 280`; `esc_neutral 307`, `esc_forward 330` (rush 345). Steering reg `0x06` (ch0), throttle reg `0x0A` (ch1). Proven on 0409/0423/demoday + AI Innovation Day.

### JetRacer Reg — I2C bus 1
| Addr | Device | Use |
|---|---|---|
| 0x3c | SSD1306 | OLED status screen |
| 0x40 | PCA9685 #1 | CH0 steering servo (reg `0x06`), inverted vs Pro |
| 0x41 | INA219 | battery / current monitor |
| 0x60 | PCA9685 #2 | TB6612 H-bridge — drive motors |
| 0x70 | (alias) | PCA9685 all-call |

Calibration: `steer_center 425`, `steer_left 300`, `steer_right 500` (inverted); steering reg `0x06` (ch0, same as Pro). Motors on 0x60 — Motor A/right `pwm0 dir1,2` ✅, Motor B/left `pwm6 dir7,8` ✅. Forward = dir_a LOW / dir_b HIGH (kit wiring reversed vs the discovery tool).

---

## D. Replication model

Validate **one car per platform**, lock that profile, then every other car of
the same model is **copy-paste** — same image, same node, same profile.

- **Uniform** (flash-identical fleet-wide): `acdc-ros2:foxy` image + the driver package.
- **Per-platform** (the only diff): which `car_config` profile is `ACTIVE`.
- **Per-individual-car** (coordination only): a unique ROS namespace, e.g. `/car_03/cmd_vel`.

"Flash the fleet" = `docker load` the image + `cp` the driver folder. It runs
**inside** the live Ubuntu 18.04 — it does **not** re-image the Jetson and is fully reversible.

---

## E. Status / open items

- **Pro** — driver built, reuses validated constants; needs one in-container drive test to sign off.
- **Reg** — both motors confirmed (right `pwm0 dir1,2`, left `pwm6 dir7,8`); `DcHbridgeDrive` implemented, `validated=True`. Needs one in-container test drive to sign off (and eyeball forward direction).
- **Verify (❓):** Pro power source/pack; whether any car will get a LiDAR (gates the guide's SLAM/Nav2); Reg wheel encoders (present, unused).
