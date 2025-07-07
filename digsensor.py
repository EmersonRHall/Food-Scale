from machine import Pin, I2C
import utime

# Touch sensor
touch = Pin(15, Pin.IN)

# I2C
i2c = I2C(0, scl=Pin(1), sda=Pin(0))
APDS9930_ADDR = 0x39

# Registers
ENABLE = 0x80
ALS_DATA0 = 0x94  # Channel 0 lower byte (ambient light)
ALS_DATA1 = 0x96  # Channel 1 lower byte (optional)

def write_register(reg, val):
    i2c.writeto_mem(APDS9930_ADDR, reg, bytes([val]))

def read_als_channel(reg):
    return int.from_bytes(i2c.readfrom_mem(APDS9930_ADDR, reg, 2), 'little')

def init_als():
    try:
        write_register(ENABLE, 0x03)  # PON + AEN = enable ALS
        print("ALS enabled.")
        return True
    except:
        print("Failed to enable ALS.")
        return False

def read_ambient_light():
    return read_als_channel(ALS_DATA0)

# Setup
print("Scanning I2C...")
devices = i2c.scan()
print("Devices found:", devices)

if init_als():
    triggered = False
    while True:
        light = read_ambient_light()
        print("Ambient Light:", light)

        # Trigger when light drops below threshold
        if light < 4 and not triggered:
            print("HAND DETECTED - TOGGLE ON")
            triggered = True
            utime.sleep(0.5)  # debounce delay
        elif light >= 5:
            triggered = False

        utime.sleep(0.2)
