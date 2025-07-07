from machine import Pin, I2C, ADC
import utime

# --- Touch sensor pins ---
touch1 = Pin(15, Pin.IN)  # Tare
touch2 = Pin(14, Pin.IN)  # Send to app
touch3 = Pin(13, Pin.IN)  # Reset tare (read absolute)

# --- I2C and ALS sensor (APDS-9930) ---
i2c = I2C(0, scl=Pin(1), sda=Pin(0))
APDS9930_ADDR = 0x39
ENABLE = 0x80
ALS_DATA0 = 0x94

# --- Load cell ADCs ---
adc1 = ADC(26)  # Load cell 1 (A0)
adc2 = ADC(27)  # Load cell 2 (A1)

# --- Kalman Filter Class ---
class KalmanFilter:
    def __init__(self):
        self.Q = 0.022
        self.R = 0.1
        self.P = 1.0
        self.K = 0.0
        self.X = 0.0

    def update(self, raw):
        self.K = self.P / (self.P + self.R)
        self.X = self.K * raw + (1 - self.K) * self.X
        self.P = (1 - self.K) * self.P + self.Q
        return self.X

kf1 = KalmanFilter()
kf2 = KalmanFilter()

# --- State ---
scale_on = False
als_triggered = False
touch1_triggered = False
touch2_triggered = False
touch3_triggered = False
tare_offset = 0.0
use_tare = True
THRESHOLD = 4
led = Pin(25, Pin.OUT)

# --- I2C Register Helpers ---
def write_register(reg, val):
    i2c.writeto_mem(APDS9930_ADDR, reg, bytes([val]))

def read_als():
    return int.from_bytes(i2c.readfrom_mem(APDS9930_ADDR, ALS_DATA0, 2), 'little')

def init_als():
    try:
        write_register(ENABLE, 0x03)
        print("Ambient light sensor enabled.")
    except:
        print("Failed to initialize ALS sensor.")

# --- Start ---
print("Scanning I2C bus...")
devices = i2c.scan()
print("I2C devices found:", devices)

init_als()

# Initialize Kalman filters with initial ADC readings
X1 = adc1.read_u16() / 65535 * 3.3
X2 = adc2.read_u16() / 65535 * 3.3
kf1.X = X1
kf2.X = X2
tare_offset = (X1 + X2) / 2

while True:
    # --- ALS ON/OFF ---
    light = read_als()
    if light < THRESHOLD and not als_triggered:
        scale_on = not scale_on
        led.value(scale_on)
        print("🖐️ Scale turned", "ON ✅" if scale_on else "OFF ❌")
        als_triggered = True
        utime.sleep(0.5)
    elif light >= THRESHOLD:
        als_triggered = False

    if scale_on:
        # --- Read current ADC values ---
        raw1 = adc1.read_u16() / 65535 * 3.3
        raw2 = adc2.read_u16() / 65535 * 3.3
        filtered1 = kf1.update(raw1)
        filtered2 = kf2.update(raw2)
        avg_filtered = (filtered1 + filtered2) / 2

        # --- Tare (Touch 1) ---
        if touch1.value() and not touch1_triggered:
            tare_offset = avg_filtered
            use_tare = True
            print("Touch 1 ➜ TARE SET to {:.3f} V".format(tare_offset))
            touch1_triggered = True
        elif not touch1.value():
            touch1_triggered = False

        # --- Send to App (Touch 2) ---
        if touch2.value() and not touch2_triggered:
            print("Touch 2 ➜ SEND TO APP")
            # Insert send logic here
            touch2_triggered = True
        elif not touch2.value():
            touch2_triggered = False

        # --- Reset Tare (Touch 3) ---
        if touch3.value() and not touch3_triggered:
            use_tare = False
            print("Touch 3 ➜ TARE RESET — RAW MODE")
            touch3_triggered = True
        elif not touch3.value():
            touch3_triggered = False

        # --- Final weight output ---
        if use_tare:
            weight = avg_filtered - tare_offset
            print("Relative Weight: {:.3f} V".format(weight))
        else:
            print("Raw Avg Voltage: {:.3f} V".format(avg_filtered))

    utime.sleep(0.1)
