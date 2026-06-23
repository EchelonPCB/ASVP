import smbus2, time

BUS  = 1
ADDR = 0x40
MODE1     = 0x00
PRESCALE  = 0xFE
LED0_ON_L = 0x06

bus = smbus2.SMBus(BUS)

def w(reg, val): bus.write_byte_data(ADDR, reg, val & 0xFF)
def r(reg): return bus.read_byte_data(ADDR, reg)

def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])

def init():
    w(MODE1, 0x00);   time.sleep(0.1)
    w(MODE1, 0x10);   time.sleep(0.1)
    w(PRESCALE, 121); time.sleep(0.1)
    w(MODE1, 0xA1);   time.sleep(0.1)

    mode1_val    = r(MODE1)
    prescale_val = r(PRESCALE)
    print(f"MODE1={hex(mode1_val)} PRESCALE={prescale_val}")

    if prescale_val != 121:
        raise RuntimeError("PRESCALE wrong. Do not proceed.")
    if mode1_val & 0x10:
        raise RuntimeError("Chip still sleeping. Do not proceed.")
    print("Init verified.")

# --- MAIN ---
init()

import signal, sys

def safe_stop(sig, frame):
    print("/Oh Shit")
    set_pwm(1,307)
    time.sleep(0.8)
    bus.close()
    sys.exit(0)

signal.signal(signal.SIGINT, safe_stop)

print("Arming ESC at neutral (307)...")
set_pwm(1, 307)
time.sleep(3.0)
print("Arm complete. Starting throttle test.")

for val in [310, 315, 320, 325, 330, 340, 350]:
    set_pwm(1, val)
    time.sleep(1.0)
    response = input(f"val={val} — what do wheels do? (spin/nothing/faster): ")

print("Returning to neutral.")
set_pwm(1, 307)
time.sleep(1.0)
bus.close()
