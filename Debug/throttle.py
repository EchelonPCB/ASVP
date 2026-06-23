import smbus2
import time

BUS  = 1
ADDR = 0x40

MODE1     = 0x00
PRESCALE  = 0xFE
LED0_ON_L = 0x06

# Confirmed calibrated values for this car
STEERING_RIGHT  = 280
STEERING_CENTER = 320
STEERING_LEFT   = 370

# ESC values — neutral is a guess, tune after arming confirmed
ESC_NEUTRAL = 307

bus = smbus2.SMBus(BUS)

def w(reg, val):
    bus.write_byte_data(ADDR, reg, val & 0xFF)

def r(reg):
    return bus.read_byte_data(ADDR, reg)

def set_pwm(ch, off):
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])

def init():
    w(MODE1, 0x00);  time.sleep(0.1)
    w(MODE1, 0x10);  time.sleep(0.1)
    w(PRESCALE, 121); time.sleep(0.1)
    w(MODE1, 0xA1);  time.sleep(0.1)

    mode1_val    = r(MODE1)
    prescale_val = r(PRESCALE)
    print(f"MODE1={hex(mode1_val)} PRESCALE={prescale_val}")

    if prescale_val != 121:
        raise RuntimeError("Init failed. Do not proceed.")
    
    if mode1_val & 0x10:
        raise RuntimeError("Chip still slumped, dont go")
    print("Init verified.")

try:
    init()

    # Steering test
    for val in [STEERING_CENTER, STEERING_RIGHT, STEERING_CENTER, STEERING_LEFT, STEERING_CENTER]:
        set_pwm(0, val)
        print(f"Steering={val}")
        time.sleep(2.0)

    # Arm the ESC — hold neutral for 2 seconds
    print("Arming ESC...")
    set_pwm(1, ESC_NEUTRAL)
    time.sleep(2.0)
    print("ESC armed. Sending tiny throttle...")

    # Tiny throttle test
    for val in range(307, 360, +10):
        set_pwm(1, 330)
        print(f"ESC val={val}")
        time.sleep(0.5)
    set_pwm(1, ESC_NEUTRAL)
    print("Back to neutral.")

except RuntimeError as e:
    print(f"INIT FAILED: {e}")

finally:
    set_pwm(0, STEERING_CENTER)
    set_pwm(1, ESC_NEUTRAL)
    bus.close()
