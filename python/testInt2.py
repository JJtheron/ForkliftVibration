import time
import lgpio
from smbus2 import SMBus

I2C_BUS = 1
ADXL343_ADDRESS = 0x53
INT2_GPIO = 17  # Wire connects physical INT2 pin on sensor to GPIO 17 (Pin 11) on Pi

# ADXL343 registers
REG_THRESH_ACT = 0x24
REG_ACT_INACT_CTL = 0x27
REG_BW_RATE = 0x2C
REG_POWER_CTL = 0x2D
REG_INT_ENABLE = 0x2E
REG_INT_MAP = 0x2F
REG_INT_SOURCE = 0x30
REG_DATA_FORMAT = 0x31
REG_DATA_X0 = 0x32

# Interrupt bit definitions
INT_SINGLE_TAP = 1 << 6
INT_ACTIVITY = 1 << 4

# Threshold set to ~0.5g (8 * 62.5mg) so it doesn't instantly latch
ACTIVITY_THRESHOLD = 0x10


def configure_sensor(bus):
    # Put the device in standby while configuring it.
    bus.write_byte_data(ADXL343_ADDRESS, REG_POWER_CTL, 0x00)

    # Full-resolution +/- 16 g range.
    bus.write_byte_data(ADXL343_ADDRESS, REG_DATA_FORMAT, 0x08)

    # 100 Hz output data rate.
    bus.write_byte_data(ADXL343_ADDRESS, REG_BW_RATE, 0x0D)

    # Activity threshold.
    bus.write_byte_data(ADXL343_ADDRESS, REG_THRESH_ACT, ACTIVITY_THRESHOLD)

    # Activity detection on X, Y, and Z axes. DC-coupled.
    bus.write_byte_data(ADXL343_ADDRESS, REG_ACT_INACT_CTL, 0x70)

    # ROUTING FIX: Set to 0xFF to force ALL interrupts (Taps & Activity) to INT2 pin
    bus.write_byte_data(ADXL343_ADDRESS, REG_INT_MAP, 0xFF)

    # Clear any pending interrupt by reading INT_SOURCE.
    bus.read_byte_data(ADXL343_ADDRESS, REG_INT_SOURCE)

    # Enable Activity and Single Tap alerts
    bus.write_byte_data(ADXL343_ADDRESS, REG_INT_ENABLE, INT_ACTIVITY | INT_SINGLE_TAP)

    # Measurement mode.
    bus.write_byte_data(ADXL343_ADDRESS, REG_POWER_CTL, 0x08)


def read_acceleration(bus):
    # Uses | 0x80 to enable multi-byte auto-increment on ADXL343
    raw = bus.read_i2c_block_data(ADXL343_ADDRESS, REG_DATA_X0 | 0x80, 6)

    values = []
    for index in range(0, 6, 2):
        value = raw[index] | (raw[index + 1] << 8)
        if value & 0x8000:
            value -= 0x10000
        values.append(value)

    # In full-resolution mode, approximately 256 counts = 1 g.
    return tuple(value / 256.0 for value in values)


def main():
    gpio_handle = lgpio.gpiochip_open(0)
    # ELECTRICAL FIX: Explicitly enable internal pull-down to prevent floating signals
    lgpio.gpio_claim_input(gpio_handle, INT2_GPIO, lgpio.SET_PULL_DOWN)

    try:
        with SMBus(I2C_BUS) as bus:
            configure_sensor(bus)

            print("ADXL343 configured.")
            print(f"Waiting for Activity on GPIO {INT2_GPIO} via sensor INT2 pin...")
            print("Gently shake or firmly tap the sensor.")

            previous_state = lgpio.gpio_read(gpio_handle, INT2_GPIO)
            print(f"Initial pin state: {previous_state}")
            current_state = previous_state
            while True:
                
                if current_state != previous_state:
                    print(f"\nINT2 pin changed: {previous_state} -> {current_state}")
                    
                    if current_state == 1:
                        print("Acceleration (g):", read_acceleration(bus))

                        # Reading INT_SOURCE clears the latched interrupt and pulls INT2 back to 0V
                        source = bus.read_byte_data(ADXL343_ADDRESS, REG_INT_SOURCE)
                        print(f"INT_SOURCE flag cleared: 0x{source:02X}")

                    previous_state = current_state
                current_state = lgpio.gpio_read(gpio_handle, INT2_GPIO)

                time.sleep(0.005)

    finally:
        lgpio.gpiochip_close(gpio_handle)


if __name__ == "__main__":
    main()
