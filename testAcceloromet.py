import time
from smbus2 import SMBus

ADDR = 0x53
BUS = 1

# ADXL343 registers
REG_DEVID = 0x00
REG_DATAX0 = 0x32
REG_DATAX1 = 0x33
REG_DATAY0 = 0x34
REG_DATAY1 = 0x35
REG_DATAZ0 = 0x36
REG_DATAZ1 = 0x37

bus = SMBus(BUS)

# confirm device
devid = bus.read_byte_data(ADDR, REG_DEVID)
print(f"DEVID=0x{devid:02X}")
if devid != 0xE5:
    raise RuntimeError(f"Unexpected device ID: 0x{devid:02X}")

# set measurement mode
bus.write_byte_data(ADDR, 0x2D, 0x08)   # POWER_CTL = measure
bus.write_byte_data(ADDR, 0x31, 0x08)   # DATA_FORMAT = full-res, +/-2g
# optional: faster output rate
bus.write_byte_data(ADDR, 0x2C, 0x0D)   # 800 Hz

try:
    while True:
        x = bus.read_i2c_block_data(ADDR, REG_DATAX0, 2)
        y = bus.read_i2c_block_data(ADDR, REG_DATAY0, 2)
        z = bus.read_i2c_block_data(ADDR, REG_DATAZ0, 2)

        x_raw = (x[1] << 8) | x[0]
        y_raw = (y[1] << 8) | y[0]
        z_raw = (z[1] << 8) | z[0]

        # convert to signed 16-bit
        if x_raw >= 0x8000:
            x_raw -= 0x10000
        if y_raw >= 0x8000:
            y_raw -= 0x10000
        if z_raw >= 0x8000:
            z_raw -= 0x10000

        # full-res +/-2g -> 1 LSB = 3.9 mg = 0.0039 g
        x_g = x_raw * 0.0039
        y_g = y_raw * 0.0039
        z_g = z_raw * 0.0039

        print(f"\rX={x_g:7.3f} g  Y={y_g:7.3f} g  Z={z_g:7.3f} g", end="", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nStopped")
finally:
    bus.close()