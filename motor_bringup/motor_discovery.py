#!/usr/bin/env python3
"""
motor_discovery.py — JetRacer (non-Pro) AI Kit drive-motor bring-up / discovery
================================================================================
Find the motor controller's I2C address and figure out HOW to actuate it.

WHY: the JetRacer *Pro* drives its wheels with an ESC on PCA9685 CH1 (a single
servo-style pulse: neutral 307 / forward 330). This car — the JetRacer *AI Kit*
— uses 37-520 *DC encoder gearmotors* through an H-bridge driver (commonly a
TB6612, often on a SECOND PCA9685 around 0x60). A DC motor will NOT spin from an
'ESC pulse'. It needs a PWM (speed) channel PLUS direction pins. This tool finds
which address/channels, and tests both the DC-motor and ESC patterns.

RUN ON THE HOST (Ubuntu 18.04 / Python 3.6 / smbus2) — simplest for raw bring-up.
  *** WHEELS OFF THE GROUND. Jetson on its OWN stable power (brownout history). ***

  sudo python3 motor_discovery.py --scan                       # find all I2C devices
  sudo python3 motor_discovery.py --dump 0x40 --bus 1          # read PCA9685 registers
  sudo python3 motor_discovery.py --probe 0x60 --bus 1         # gently pulse each channel
  sudo python3 motor_discovery.py --servo 0x40 --bus 1 --ch 0  # confirm the steering channel
  sudo python3 motor_discovery.py --dcmotor 0x60 --bus 1 --pwm 0 --dir 1 2   # TB6612-style test
  sudo python3 motor_discovery.py --esc 0x40 --bus 1 --ch 1    # ESC-style test (Pro pattern)
"""
import argparse
import signal
import sys
import time

try:
    import smbus2
except ImportError:
    sys.exit("smbus2 missing.  Host:  sudo apt install python3-smbus  ||  sudo pip3 install smbus2")

# ── PCA9685 registers ──────────────────────────────────────────────────────
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

# Servo-SAFE probe value (~2 ms pulse @ 50 Hz). Within the AI Kit range (300-500)
# so it cannot slam a steering servo to a mechanical bind. ~10% duty for a motor.
PROBE_VAL = 400
DCMOTOR_MAX = 1400          # max PWM ticks for the DC-motor test (gentle ~34% duty)

_bus = None


def _safe_exit(*_a):
    global _bus
    print("\n[safe] all channels off")
    try:
        if _bus is not None:
            for ch in range(16):
                _bus.write_i2c_block_data(_bus._addr_hint, LED0_ON_L + 4 * ch, [0, 0, 0, 0]) \
                    if hasattr(_bus, "_addr_hint") else None
    except Exception:
        pass
    sys.exit(0)


def reg_w(bus, addr, reg, val):
    bus.write_byte_data(addr, reg, val & 0xFF)


def reg_r(bus, addr, reg):
    return bus.read_byte_data(addr, reg)


def set_pwm(bus, addr, ch, on, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(addr, base, [on & 0xFF, on >> 8, off & 0xFF, off >> 8])


def ch_off(bus, addr, ch):
    set_pwm(bus, addr, ch, 0, 0)


def all_off(bus, addr):
    for ch in range(16):
        ch_off(bus, addr, ch)


def digital(bus, addr, ch, high):
    """Drive a channel as a near-digital level (for H-bridge direction pins)."""
    set_pwm(bus, addr, ch, 0, 4095 if high else 0)


def init_pca(bus, addr, freq=50):
    prescale = int(round(25_000_000.0 / (4096 * freq)) - 1)   # 50 Hz -> 121
    reg_w(bus, addr, MODE1, 0x00); time.sleep(0.05)
    reg_w(bus, addr, MODE1, 0x10); time.sleep(0.05)           # sleep to set prescale
    reg_w(bus, addr, PRESCALE, prescale); time.sleep(0.05)
    reg_w(bus, addr, MODE1, 0xA1); time.sleep(0.05)           # wake + auto-inc + allcall
    rb = reg_r(bus, addr, PRESCALE)
    print(f"[init] 0x{addr:02X}: prescale set {prescale}, readback {rb}  "
          f"(~{25_000_000/(4096*(rb+1)):.0f} Hz)")
    if rb != prescale:
        print("       WARNING: prescale readback mismatch — wrong device or bus?")
    return prescale


def _guess(a):
    if a == 0x40:
        return "PCA9685 default  -> likely STEERING servo"
    if 0x41 <= a <= 0x47:
        return "PCA9685 (alt)    -> servo / steering?"
    if 0x60 <= a <= 0x6F:
        return "PCA9685 @0x6x    -> likely the MOTOR driver (JetBot/Adafruit style)"
    if a == 0x70:
        return "PCA9685 all-call"
    if a == 0x68:
        return "maybe IMU / RTC"
    return "unknown device"


def scan(buses=(0, 1)):
    print("Scanning I2C... (if a bus shows nothing, also try:  i2cdetect -y -r <bus>)")
    for b in buses:
        try:
            bus = smbus2.SMBus(b)
        except Exception as e:
            print(f"\n== bus {b}: cannot open ({e}) ==")
            continue
        found = []
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                found.append(addr)
            except Exception:
                try:
                    bus.write_quick(addr)
                    found.append(addr)
                except Exception:
                    pass
        bus.close()
        print(f"\n== I2C bus {b} ==")
        if not found:
            print("  (no devices found)")
        for a in found:
            print(f"  0x{a:02X}   {_guess(a)}")
    print("\nNext: --dump each address, then --probe the suspected motor address.")


def dump(bus_n, addr):
    bus = smbus2.SMBus(bus_n)
    try:
        m1 = reg_r(bus, addr, MODE1)
        m2 = reg_r(bus, addr, MODE2)
        pre = reg_r(bus, addr, PRESCALE)
        hz = 25_000_000 / (4096 * (pre + 1)) if pre else 0
        print(f"0x{addr:02X} (bus {bus_n}):  MODE1=0x{m1:02X}  MODE2=0x{m2:02X}  "
              f"PRESCALE={pre} (~{hz:.0f} Hz)")
        print("  ch :   ON    OFF   (current outputs)")
        for ch in range(16):
            d = bus.read_i2c_block_data(addr, LED0_ON_L + 4 * ch, 4)
            on = d[0] | (d[1] << 8)
            off = d[2] | (d[3] << 8)
            flag = "  <- active" if off not in (0, 4096) and off != 0 else ""
            print(f"  {ch:2d} : {on:5d}  {off:5d}{flag}")
    except Exception as e:
        print(f"  read failed: {e}  (wrong address/bus, or not a PCA9685)")
    finally:
        bus.close()


def probe(bus_n, addr):
    global _bus
    bus = smbus2.SMBus(bus_n); _bus = bus; _bus._addr_hint = addr
    init_pca(bus, addr)
    print(f"\nProbing 0x{addr:02X}: each channel gets a gentle pulse ({PROBE_VAL}).")
    print("Watch the car. Note which channel moves the STEERING, and whether any")
    print("channel makes the MOTOR twitch (DC motors usually need direction pins too).")
    print("Ctrl+C to abort.\n")
    for ch in range(16):
        input(f"  [Enter] to pulse channel {ch:2d} ...")
        set_pwm(bus, addr, ch, 0, PROBE_VAL)
        time.sleep(1.0)
        ch_off(bus, addr, ch)
        print(f"     channel {ch:2d}: pulsed, now off.")
    all_off(bus, addr)
    bus.close()
    print("\nProbe done. Record the steering channel and any motor reaction.")


def servo(bus_n, addr, ch, lo=300, hi=500, center=425):
    """Confirm a STEERING channel by sweeping it inside the AI-Kit safe range."""
    global _bus
    bus = smbus2.SMBus(bus_n); _bus = bus; _bus._addr_hint = addr
    init_pca(bus, addr)
    print(f"\nServo sweep on 0x{addr:02X} ch{ch}  ({lo}<-{center}->{hi}). Wheels can be down.")
    for off in [center, lo, center, hi, center]:
        print(f"  -> {off}")
        set_pwm(bus, addr, ch, 0, off)
        time.sleep(0.8)
    ch_off(bus, addr, ch)
    bus.close()
    print("If the front wheels turned left/right and re-centered, this is the steering channel.")


def dcmotor(bus_n, addr, pwm_ch, dir_a, dir_b):
    """TB6612-style DC-motor test: set direction pins, ramp the PWM channel."""
    global _bus
    bus = smbus2.SMBus(bus_n); _bus = bus; _bus._addr_hint = addr
    init_pca(bus, addr)
    print(f"\n*** DC-MOTOR TEST on 0x{addr:02X}: PWM ch{pwm_ch}, dir ch{dir_a}/{dir_b} ***")
    print("*** WHEELS OFF THE GROUND ***")
    for n in range(4, 0, -1):
        print(f"   start in {n}...  (Ctrl+C aborts)"); time.sleep(1)
    try:
        print("[fwd] dir A=high B=low, ramp PWM up")
        digital(bus, addr, dir_a, True)
        digital(bus, addr, dir_b, False)
        for v in range(0, DCMOTOR_MAX + 1, 100):
            set_pwm(bus, addr, pwm_ch, 0, v); time.sleep(0.15)
        time.sleep(1.0)
        print("[stop]"); set_pwm(bus, addr, pwm_ch, 0, 0); time.sleep(0.5)
        print("[rev] dir A=low B=high, ramp PWM up")
        digital(bus, addr, dir_a, False)
        digital(bus, addr, dir_b, True)
        for v in range(0, DCMOTOR_MAX + 1, 100):
            set_pwm(bus, addr, pwm_ch, 0, v); time.sleep(0.15)
        time.sleep(1.0)
    finally:
        all_off(bus, addr); bus.close()
    print("\nIf a wheel spun forward then reverse: FOUND IT. Record pwm/dir channels in car_config.py.")
    print("No spin? try other dir-channel pairs, or the motor driver needs a STBY pin high,")
    print("or external motor power isn't connected (logic power != motor power). See the procedure doc.")


def esc(bus_n, addr, ch):
    """ESC-style test (the JetRacer Pro pattern) — rule it in or out on this car."""
    global _bus
    bus = smbus2.SMBus(bus_n); _bus = bus; _bus._addr_hint = addr
    init_pca(bus, addr)
    print(f"\nESC-style test on 0x{addr:02X} ch{ch}. *** WHEELS OFF GROUND ***")
    print("[arm] neutral 307 for 2 s"); set_pwm(bus, addr, ch, 0, 307); time.sleep(2)
    print("[fwd] 330 for 1.5 s"); set_pwm(bus, addr, ch, 0, 330); time.sleep(1.5)
    print("[neutral]"); set_pwm(bus, addr, ch, 0, 307); time.sleep(0.5)
    ch_off(bus, addr, ch); bus.close()
    print("Spun? this car has an ESC after all. No spin (and it's a DC-gearmotor kit)? expected — use --dcmotor.")


def main():
    signal.signal(signal.SIGINT, _safe_exit)
    signal.signal(signal.SIGTERM, _safe_exit)
    p = argparse.ArgumentParser(description="JetRacer AI Kit motor discovery / bring-up")
    p.add_argument("--scan", action="store_true")
    p.add_argument("--dump", metavar="ADDR")
    p.add_argument("--probe", metavar="ADDR")
    p.add_argument("--servo", metavar="ADDR")
    p.add_argument("--esc", metavar="ADDR")
    p.add_argument("--dcmotor", metavar="ADDR")
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--ch", type=int, default=0)
    p.add_argument("--pwm", type=int)
    p.add_argument("--dir", type=int, nargs=2, metavar=("A", "B"))
    a = p.parse_args()

    def h(x):
        return int(x, 16) if isinstance(x, str) and x.lower().startswith("0x") else int(x)

    if a.scan:
        scan()
    elif a.dump:
        dump(a.bus, h(a.dump))
    elif a.probe:
        probe(a.bus, h(a.probe))
    elif a.servo:
        servo(a.bus, h(a.servo), a.ch)
    elif a.esc:
        esc(a.bus, h(a.esc), a.ch)
    elif a.dcmotor:
        if a.pwm is None or not a.dir:
            sys.exit("--dcmotor needs  --pwm <ch>  --dir <A> <B>")
        dcmotor(a.bus, h(a.dcmotor), a.pwm, a.dir[0], a.dir[1])
    else:
        p.print_help()


if __name__ == "__main__":
    main()
