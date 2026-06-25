# ACDC JetRacer Fleet — Full Spec Sheet (Pro vs Reg)

The fleet is **two different JetRacer platforms** that drive the wheels through
**completely different hardware.** Anything that hardcodes one silently fails on
the other (this is what bit the motor bring-up). This is the authoritative
distinction — keep it in sync with `car_config.py`.

> **Naming:** Waveshare's **"JetRacer AI Kit"** = the regular **Reg** car here;
> **"JetRacer Pro AI Kit"** = **Pro**. Older repo docs say "AI Kit (non-Pro)" —
> same car as **Reg**. Legend: ✅ confirmed · ❓ verify.

---

## A. Universal — identical on both (lives in the image + driver, written once)

| Layer | Spec |
|---|---|
| Compute | Jetson Nano Dev Kit, **4 GB (B01)** |
| OS / SDK | Ubuntu **18.04.6** · JetPack **4.x** (4.5.1 / R32.5.2 verified on the Reg car) |
| CUDA / Python (host) | CUDA **10.2** · Python **3.6** |
| I2C | **bus 1** (bus 0 dead on both) |
| Steering controller | PCA9685 **@ 0x40, channel 0** · prescale **121 → 50 Hz** · raw `smbus2` |
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
| Drive hardware | **ESC** (brushless-style) | **2× 37-520 DC encoder gearmotors** |
| Drive controller | PCA9685 **0x40, CH1** (same chip as steering) | **2nd PCA9685 @ 0x60** + TB6612 H-bridge |
| Throttle command | ESC pulse — neutral **307**, fwd **325–330** | PWM + direction pins, per motor |
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
| 0x40 | PCA9685 | CH0 steering servo · CH1 ESC throttle |
| 0x70 | (alias) | PCA9685 all-call (appears once initialized) |

Calibration: `steer_center 320`, `steer_left 370`, `steer_right 280`; `esc_neutral 307`, `esc_forward 325`. Proven on 0409/0423/demoday + AI Innovation Day.

### JetRacer Reg — I2C bus 1
| Addr | Device | Use |
|---|---|---|
| 0x3c | SSD1306 | OLED status screen |
| 0x40 | PCA9685 #1 | CH0 steering servo (inverted vs Pro) |
| 0x41 | INA219 | battery / current monitor |
| 0x60 | PCA9685 #2 | TB6612 H-bridge — drive motors |
| 0x70 | (alias) | PCA9685 all-call |

Calibration: `steer_center 425`, `steer_left 300`, `steer_right 500` (inverted); motors on 0x60 — Motor A/right `pwm0 dir1,2` ✅, Motor B/left `pwm6 dir7,8` ✅. Forward = dir_a LOW / dir_b HIGH (kit wiring reversed vs the discovery tool).

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
