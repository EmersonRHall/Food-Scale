from machine import Pin, I2C
import utime

# --- Touch sensor pins ---
touch1 = Pin(15, Pin.IN)  # Tare
touch2 = Pin(14, Pin.IN)  # Send to app

# --- I2C and ALS sensor (APDS-9930) ---
i2c = I2C(0, scl=Pin(1), sda=Pin(0))
APDS9930_ADDR = 0x39
ENABLE = 0x80
ALS_DATA0 = 0x94  # Ambient Light Channel 0

# --- Internal state ---
scale_on = False
als_triggered = False
touch1_triggered = False
touch2_triggered = False
THRESHOLD = 400  # ALS hand wave threshold
led = Pin(25, Pin.OUT)  # Pico onboard LED to show ON/OFF state

# --- I2C register helpers ---
def write_register(reg, val):
    i2c.writeto_mem(APDS9930_ADDR, reg, bytes([val]))

def read_als():
    return int.from_bytes(i2c.readfrom_mem(APDS9930_ADDR, ALS_DATA0, 2), 'little')

# --- Initialize ALS sensor ---
def init_als():
    try:
        write_register(ENABLE, 0x03)  # Power ON + ALS enable
        print("Ambient light sensor enabled.")
    except:
        print("Failed to initialize ALS sensor.")

# --- Start ---
print("Scanning I2C bus...")
devices = i2c.scan()
print("I2C devices found:", devices)

init_als()

while True:
    # --- Read ALS ---
    light = read_als()
    # print("Ambient Light:", light)

    # --- Hand wave: toggle scale on/off ---
    if light < THRESHOLD and not als_triggered:
        scale_on = not scale_on
        led.value(scale_on)  # Light the onboard LED when ON
        print("🖐️ Scale turned", "ON ✅" if scale_on else "OFF ❌")
        als_triggered = True
        utime.sleep(0.5)  # Debounce

    elif light >= THRESHOLD:
        als_triggered = False  # Reset when hand is gone

    # --- Touch Sensor 1: TARE ---
    if scale_on:
        t1 = touch1.value()
        t2 = touch2.value()

        if t1 and not touch1_triggered:
            print("Touch 1 ➜ TARE SCALE")
            touch1_triggered = True
            # 👉 Insert tare logic here

        elif not t1:
            touch1_triggered = False  # Reset on release

        # --- Touch Sensor 2: SEND TO APP ---
        if t2 and not touch2_triggered:
            print("Touch 2 ➜ SEND WEIGHT TO APP")
            touch2_triggered = True
            # 👉 Insert send logic here

        elif not t2:
            touch2_triggered = False  # Reset on release

    utime.sleep(0.05)
