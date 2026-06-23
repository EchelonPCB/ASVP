# Operating Procedure — Drive-Motor Bring-Up (JetRacer AI Kit, non-Pro)

| Field | Detail |
|---|---|
| Project | ACDC JetRacer Pro Autonomous (JPA) |
| Subsystem | Drive-motor actuation · I2C discovery · power |
| Car | **Waveshare JetRacer AI Kit (non-Pro)** — *not* the JetRacer Pro |
| Host | Jetson Nano · Ubuntu 18.04 · JetPack 4.x · Python 3.6 |
| Goal | Find the drive-motor controller address + actuate the wheels |
| Tools | `motor_discovery.py`, `car_config.py` (this folder) |

---

## 1. The core problem (read first — it changes your whole approach)
You have **two different kits** and they drive the wheels **differently**:

| | JetRacer **Pro** (your 6 cars, `0409`/`0423`) | JetRacer **AI Kit** (this new car) |
|---|---|---|
| Drive hardware | **ESC** (brushless-style) | **37-520 DC encoder gearmotors** |
| Control | one PWM channel, servo-style pulse (neutral 307 / fwd 330) | **H-bridge driver** (likely TB6612): **PWM + direction pins** |
| Address | PCA9685 @ **0x40**, CH1 | motor driver likely a **2nd PCA9685 ~0x60** |
| Steering | center 320, L 370 / R 280 (higher=left) | center 425, **L 300 / R 500** (lower=left — *inverted*) |

**This is why the motor won't actuate:** you're sending it the *Pro's* ESC pulse,
but a DC gearmotor **cannot spin from a single pulse** — it needs a **PWM (speed)
channel AND direction pins**, on a driver that is probably **at a different I2C
address you haven't found yet.** Stop thinking "ESC"; think "DC motor + H-bridge."

---

## 2. SAFETY — do this before anything moves
You already browned out the Jetson twice. That is a **power** fault, not code.
- **Wheels OFF the ground** for every test below.
- **Jetson on its own stable 5 V / UPS** — *not* the servo/motor BEC.
- **Separate motor/servo power rail** with enough current; **common ground**
  between Jetson, the PCA/driver board, and the motor supply.
- Confirm **motor power is actually connected** — H-bridge logic power ≠ motor
  power. A driver can ACK on I2C and set pins while the motors have no voltage.
- Keep a finger on **Ctrl+C**; every mode in `motor_discovery.py` powers all
  channels off on exit.

---

## 3. The procedure (tomorrow, in order)

> Do this **on the host** (Python 3.6 + smbus2) — simplest for raw bring-up. You
> do NOT need Docker/ROS 2 for motor bring-up; that's the ROS 2 brain layer, later.
> If you must do it in the container, launch it with `--device /dev/i2c-0
> --device /dev/i2c-1` (or `--privileged -v /dev:/dev`).

**Step 0 — deps**
```bash
sudo apt install -y i2c-tools python3-smbus || sudo pip3 install smbus2
scp -r motor_bringup jetson@<CAR_IP>:~/          # from your PC
ssh jetson@<CAR_IP> "sed -i 's/\r$//' ~/motor_bringup/*.py"
```

**Step 1 — find every device on BOTH buses** (don't assume 0x40, and on THIS car
don't assume bus 0 is dead):
```bash
i2cdetect -y -r 0
i2cdetect -y -r 1
sudo python3 ~/motor_bringup/motor_discovery.py --scan
```
Record every address. Expect a **0x40** (steering) and very likely a **second
device ~0x60** (the motor driver). *That 0x60-ish address is your missing motor.*

**Step 2 — confirm which device is the steering** (the one you already know moves):
```bash
sudo python3 ~/motor_bringup/motor_discovery.py --servo 0x40 --bus 1 --ch 0
```
Front wheels turn L/R and recenter -> that's steering. Now you know the **other**
address is the motor.

**Step 3 — dump the suspected motor driver's registers** (state + frequency):
```bash
sudo python3 ~/motor_bringup/motor_discovery.py --dump 0x60 --bus 1
```

**Step 4 — map the motor channels by probing** (gentle, servo-safe pulse each ch):
```bash
sudo python3 ~/motor_bringup/motor_discovery.py --probe 0x60 --bus 1
```
A DC motor usually won't spin from this alone (no direction pins) — you're looking
for which channels twitch the motor / are wired to it. Note candidates.

**Step 5 — actuate it as a DC motor** (the real test). Try a PWM channel + a pair
of direction channels (TB6612 pattern). Start with common JetBot mappings and
adjust:
```bash
# wheels off ground!  try combos until a wheel spins fwd then reverse:
sudo python3 ~/motor_bringup/motor_discovery.py --dcmotor 0x60 --bus 1 --pwm 0 --dir 1 2
sudo python3 ~/motor_bringup/motor_discovery.py --dcmotor 0x60 --bus 1 --pwm 5 --dir 3 4
```
When a wheel spins forward then reverses, **you found it.** Record `pwm`/`dir`
channels + the address into `car_config.py` under `jetracer_ai`.

**Step 6 — (only if Step 5 fails everywhere) rule out an ESC** on the steering chip:
```bash
sudo python3 ~/motor_bringup/motor_discovery.py --esc 0x40 --bus 1 --ch 1
```

---

## 4. Different paths (decision tree)
- **A — 2nd PCA9685 at ~0x60 (most likely).** It's the H-bridge driver. Use
  `--dcmotor`. This is the JetBot/Waveshare-non-Pro norm.
- **B — only 0x40 present.** The TB6612 is driven by *extra channels* on the same
  chip. Probe channels 2-7; use `--dcmotor 0x40 --pwm <ch> --dir <a> <b>`.
- **C — it really is an ESC.** Rare for a DC-gearmotor kit, but `--esc` confirms.
- **D — motor is on Jetson GPIO, not I2C.** Then i2c scan won't show it; the
  driver's PWM/DIR pins go to the 40-pin header (use `Jetson.GPIO` /
  `sysfs` PWM). Check the JetRacer-AI expansion-board pinout.
- **E — FAST PATH: use the kit's official software.** The kit ships working code.
  `pip3 install jetracer` then try `from jetracer.nvidia_racecar import
  NvidiaRacecar; car = NvidiaRacecar(); car.throttle = 0.2` — and read the
  **Waveshare JetRacer AI Kit Wiki** for the exact addresses/channels. Use this to
  *learn the mapping*, then reproduce it raw with smbus2 for full control.

---

## 5. Failure modes to expect
| Symptom | Likely cause | Action |
|---|---|---|
| Jetson reboots when motor/servo loads | shared/weak power rail | separate Jetson power + motor rail + common ground (Sec. 2) |
| Driver ACKs on I2C but nothing spins | direction pins not set, or **STBY/enable pin low**, or **no motor power** | set both dir pins, find/hold STBY high, verify motor supply voltage |
| Steering slams to a stop / binds | wrong calibration (used Pro values on AI car) | use AI Kit values: center 425, L 300, R 500, **inverted** |
| Nothing on the I2C scan | wrong bus, or motor is on GPIO not I2C | scan bus 0 AND 1; check Path D |
| Spins one direction only | one direction channel wrong/floating | swap the `--dir A B` pair |
| `--dcmotor` does nothing on any combo | mapping is on the other chip, or GPIO | try the other address; check Path B/D/E |

---

## 6. Ubuntu 18.04 / package / library limits (what will bite you)
- **Python 3.6 on the host.** No `float | None` syntax (use `Optional[...]`), and
  many modern libs dropped 3.6. `smbus2` works fine and is the right raw tool.
- **`Adafruit_PCA9685` (old) vs CircuitPython (`adafruit-circuitpython-servokit`).**
  The CircuitPython stack needs **Blinka**, which is finicky on JetPack 4 / 18.04.
  For bring-up, **prefer raw `smbus2`** (what this tool uses) — no Blinka, no
  version roulette.
- **`NvidiaRacecar` / `ServoKit` can be hard-wired to a bus/address** (your Pro
  code excluded them for that reason). They may target bus 0 or 0x40 only. Don't
  trust them to "just work" on this kit — verify with the scan first.
- **Bus number is not portable.** Pro = bus 1 (bus 0 dead). This AI Kit may differ
  — always scan both.
- **Docker note:** the container is Python 3.8 (nice) but is a *generic* userspace
  — for I2C you must pass `--device /dev/i2c-*`, and the **camera/CUDA still need
  the l4t image**. For pure motor bring-up, the **host is simpler** — do it there
  first, move to ROS 2 once the mapping is known.
- **`i2c` permissions:** if you get permission errors, run with `sudo` or add the
  user to the `i2c` group (`sudo usermod -aG i2c $USER`, re-login).

---

## 7. Definition of done / report back
Done when: a wheel spins **forward and reverse** under software control, and you've
recorded in `car_config.py`: **motor address, PWM channel, direction channels**
(+ STBY if any). Then we wrap it into a `set_throttle()` that mirrors the Pro's
interface so `demoday.py` / the ROS 2 `/cmd_vel` node work on this car unchanged.

**Paste back:** the full `--scan` output (both buses), the `--dump` of the motor
address, and which `--dcmotor` combo (if any) spun a wheel. That's everything I
need to lock the mapping and write the driver.
