import time
import struct
from smbus2 import SMBus
import lgpio

# ADXL343 I2C Address (usually 0x53, or 0x1D if ALT ADDRESS pin is pulled high)
ADXL343_ADDR = 0x53

# Register Maps
REG_INT_ENABLE  = 0x2E
REG_INT_MAP     = 0x2F
REG_INT_SOURCE  = 0x30
REG_DATAX0      = 0x32  # Start of 6-byte X, Y, Z data block
REG_FIFO_CTL    = 0x38
REG_FIFO_STATUS = 0x39
INT1_GPIO = 17

# Initialize I2C Bus (Use bus 1 for modern Raspberry Pi models)
bus = SMBus(1)

def setup_adxl343():
    # 1. Put device into standby to configure safely
    bus.write_byte_data(ADXL343_ADDR, 0x2D, 0x00) # POWER_CTL
    
    # 2. Configure FIFO: Stream Mode (0x40) + 16 samples (0x10) = 0x50
    bus.write_byte_data(ADXL343_ADDR, REG_FIFO_CTL, 0x50)
    
    # 3. Route Watermark interrupt to INT1 pin (0x00)
    bus.write_byte_data(ADXL343_ADDR, REG_INT_MAP, 0x00)
    
    # 4. Enable Watermark Interrupt (Bit 1 = 0x02)
    bus.write_byte_data(ADXL343_ADDR, REG_INT_ENABLE, 0x02)
    
    # 5. Put device into Measure Mode
    bus.write_byte_data(ADXL343_ADDR, 0x2D, 0x08)
    print("ADXL343 Watermark Configured Successfully.")

def read_fifo_batch():
    samples = []
    
    # Read FIFO_STATUS to see exactly how many samples are available
    fifo_status = bus.read_byte_data(ADXL343_ADDR, REG_FIFO_STATUS)
    entries_available = fifo_status & 0x3F # Bits 5-0 give the count
    print(f"Entries available: {entries_available}, {fifo_status}")
    bus.read_byte_data(ADXL343_ADDR, REG_INT_SOURCE)  # Clear any pending interrupts
    data = bus.read_i2c_block_data(ADXL343_ADDR, REG_DATAX0 | 0x80, 32)
    fifo_status = bus.read_byte_data(ADXL343_ADDR, REG_FIFO_STATUS)
    entries_available = fifo_status & 0x3F # Bits 5-0 give the count
    print(f"Entries available2: {entries_available}, {fifo_status}")
    # Only read if we have met or exceeded our 16-sample target
    if entries_available >= 16:
        print(f"Watermark hit! Processing {entries_available} samples...")

         # Clear the interrupt by reading the GPIO pin
        for _ in range(entries_available):
            # Burst read 6 bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
            # This automatically pops the oldest sample out of the FIFO
            data = bus.read_i2c_block_data(ADXL343_ADDR, REG_DATAX0 | 0x80, 6)
            
            # Unpack the 6 bytes into three signed 16-bit integers (<hhh)
            x, y, z = struct.unpack('<hhh', bytes(data))
            samples.append((x, y, z))
        print(samples)  # Print each sample as it's read
        # The interrupt pin on the ADXL343 clears itself automatically 
        # now because the FIFO entry count dropped below 16.
        bus.read_byte_data(ADXL343_ADDR, REG_INT_SOURCE)  # Clear any pending interrupts
        fifo_status = bus.read_byte_data(ADXL343_ADDR, REG_FIFO_STATUS)
        entries_available = fifo_status & 0x3F # Bits 5-0 give the count
        print(f"Entries available3: {entries_available}, {fifo_status}")
        time.sleep(1)
        return samples
    return None

# Main loop
try:
    setup_adxl343()
    handle = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(handle, INT1_GPIO, lgpio.SET_PULL_DOWN)
    while True:
        # If you are using a hardware GPIO interrupt pin, hook this function 
        # to a GPIO falling/rising edge detection. Otherwise, poll it:
        print(lgpio.gpio_read(handle, INT1_GPIO))  # Polling the GPIO pin for interrupt status
        batch = read_fifo_batch()
        if batch:
            print(f"Captured batch of {len(batch)} samples.")
            print(f"First Sample X,Y,Z: {batch[0]}")
            print("-" * 30)
            
        time.sleep(0.01) # Short sleep to avoid maxing out CPU during polling

except KeyboardInterrupt:
    print("Stopping script.")
