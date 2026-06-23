import smbus2
import time

# ─────────────────────────────────────────────
# WHAT THIS SCRIPT DOES (plain english)
#
# The PCA9685 is a chip that generates PWM signals —
# pulses of electricity that tell the steering servo
# where to point. This script:
#   1. Opens a communication line to that chip (I2C bus)
#   2. Configures it to pulse at 50 times per second (50Hz)
#      — the speed that RC servos expect
#   3. Verifies the configuration actually took
#   4. Sends a series of steering positions to channel 0
#
# If the configuration fails, it stops immediately
# and tells you — rather than running silently and
# doing nothing, which was the old behavior.
# ─────────────────────────────────────────────

# I2C bus 1 is the communication line the Jetson Nano
# uses to talk to the PCA9685 chip.
# Bus 0 exists but is the wrong one — never use it.
BUS  = 1

# 0x40 is the "address" of the PCA9685 chip on that bus.
# Think of it like a house number on a street.
# Confirmed present via i2cdetect -y -r 1.
ADDR = 0x40

# These are register addresses INSIDE the PCA9685 chip.
# Registers are like small mailboxes — you write a value
# into one to change the chip's behavior.
MODE1     = 0x00   # controls chip state (sleeping, awake, etc.)
PRESCALE  = 0xFE   # controls the output frequency (how fast it pulses)
LED0_ON_L = 0x06   # starting address of the first PWM channel's output registers

# Open the I2C communication line to the chip
bus = smbus2.SMBus(BUS)


def w(reg, val):
    # Write a single byte value into a register on the chip.
    # reg  = which register (mailbox) to write to
    # val  = what value to put in it
    # The & 0xFF ensures we never accidentally send
    # more than 1 byte (clips to 0–255).
    bus.write_byte_data(ADDR, reg, val & 0xFF)


def r(reg):
    # Read back whatever value is currently in a register.
    # We use this to verify our writes actually worked.
    return bus.read_byte_data(ADDR, reg)


def set_pwm(ch, off):
    # Set the PWM output for one channel (ch = channel number).
    # "off" is the pulse width value — a number from 0 to 4095
    # that maps to a pulse length between 0ms and 20ms.
    # For RC servos at 50Hz:
    #   ~270 = full right
    #   ~307 = center
    #   ~345 = full left
    # (these are approximate — calibration sweep needed)
    #
    # Each channel uses 4 bytes in the chip's memory:
    #   ON_LOW, ON_HIGH  — when the pulse turns ON  (we always use 0)
    #   OFF_LOW, OFF_HIGH — when the pulse turns OFF (this is our "off" value)
    base = LED0_ON_L + 4 * ch
    bus.write_i2c_block_data(ADDR, base, [0, 0, off & 0xFF, off >> 8])


def init():
    # ── INITIALIZATION SEQUENCE ──────────────────────────
    # The PCA9685 must be configured in a specific order.
    # Skipping steps or not waiting long enough causes
    # the configuration to silently fail — the chip appears
    # to accept your settings but ignores them internally.
    # That is why the servo sometimes worked and sometimes didn't.

    # Step 1: fully wake the chip and clear any leftover state
    # from a previous run. Writing 0x00 to MODE1 means:
    # "no sleep, no special modes, clean slate."
    w(MODE1, 0x00)
    time.sleep(0.1)  # wait 100ms — give chip time to fully reset

    # Step 2: put the chip to sleep.
    # This sounds backwards, but the PCA9685 REQUIRES the chip
    # to be in sleep mode before you're allowed to change the
    # frequency (prescale). If you write prescale while awake,
    # the chip ignores it silently. 0x10 sets the SLEEP bit.
    w(MODE1, 0x10)
    time.sleep(0.1)  # wait 100ms — chip must be fully asleep before next step

    # Step 3: write the frequency (prescale = 121 → 50Hz).
    # 50Hz means the chip sends 50 pulses per second.
    # RC servos and ESCs expect exactly this frequency.
    # Prescale value of 121 is calculated from the chip's
    # internal 25MHz oscillator to hit 50Hz.
    # This ONLY works correctly if the chip is sleeping (step 2).
    w(PRESCALE, 121)
    time.sleep(0.1)  # wait for the write to settle

    # Step 4: wake the chip back up and enable "auto-increment."
    # 0xA1 = 10100001 in binary:
    #   bit 7 (RESTART) = 1  → restart PWM channels
    #   bit 5 (AI)      = 1  → auto-increment register address
    #                          so we can write all 4 bytes of a
    #                          channel in one shot (used in set_pwm)
    #   bit 4 (SLEEP)   = 0  → chip is awake
    w(MODE1, 0xA1)
    time.sleep(0.1)  # wait for chip to fully wake before we read back

    # Step 5: read back the registers to verify our writes actually took.
    # This is the step that was NEVER done before — and is why
    # the intermittent failure was never caught.
    # If the values don't match, we stop immediately.
    mode1_val    = r(MODE1)
    prescale_val = r(PRESCALE)

    print(f"MODE1    = 0x{mode1_val:02X}  (expected 0xA1)")
    print(f"PRESCALE = {prescale_val}       (expected 121)")

    if prescale_val != 121:
        # Prescale didn't take — the chip was probably not fully
        # asleep when we wrote it. This was the suspected root cause
        # of all previous intermittent failures.
        raise RuntimeError(
            f"PRESCALE write failed — got {prescale_val}, expected 121. "
            f"Do not proceed. Check delays and retry."
        )

    if mode1_val & 0x10:
        # MODE1 didn't match — chip may be in an unexpected state.
        raise RuntimeError(
            f"MODE1 write failed — got 0x{mode1_val:02X}, expected 0xA1. "
            f"Do not proceed."
        )

    print("Init verified. Chip is configured correctly. Safe to actuate.")


# ── MAIN EXECUTION ───────────────────────────────────────
try:
    # Run the initialization sequence.
    # If it fails, the RuntimeError below catches it
    # and we never send any PWM commands.
    init()

    # Send a series of steering positions to channel 0 (steering servo).
    # These values are NOT yet calibrated for this car —
    # they are starting estimates based on the JetRacer Pro typical range.
    # A calibration sweep is needed to find the real center/left/right.
    # Channel 1 (ESC/throttle) is intentionally not touched yet.
    print("Sending steering positions...")
    for val in [280, 290, 300, 310, 320, 330, 340, 350, 360, 370]:
        set_pwm(0, val)   # channel 0 = steering
        print(f"  Steering set to off={val}")
        time.sleep(1.8)   # hold each position for 0.8 seconds

    # Return to approximate center before exiting.
    # The PCA9685 holds the last value after the script ends,
    # so explicitly centering prevents the servo from buzzing
    # under load while held at an off-center position.
    set_pwm(0, 320)
    print("Returned to center. Done.")

except RuntimeError as e:
    # Init failed — print exactly what went wrong.
    # This means no PWM was sent to the servo.
    print(f"\nINIT FAILED — stopping before actuation.\nReason: {e}")

finally:
    # Always close the I2C bus connection when done,
    # whether the script succeeded or failed.
    bus.close()
