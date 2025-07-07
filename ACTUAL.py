import network
import socket
import time
import machine
import json

# === WIFI CONFIG ===
#SSID = 'SpectrumSetup-F0CD'
#PASSWORD = 'friendlyring983'

# === WIFI CONFIG (Eduroam) ===
SSID = 'Emerson'  
PASSWORD = '11111111'  


# === SENSOR SETUP ===
adc1 = machine.ADC(26)
adc2 = machine.ADC(27)
touch1 = machine.Pin(15, machine.Pin.IN)
touch2 = machine.Pin(14, machine.Pin.IN)
touch3 = machine.Pin(13, machine.Pin.IN)
led = machine.Pin(25, machine.Pin.OUT)

# === I2C LIGHT SENSOR (APDS-9930) ===
i2c = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0))
APDS9930_ADDR = 0x39
ENABLE = 0x80
ALS_DATA0 = 0x94
THRESHOLD = 2

def write_register(reg, val):
    i2c.writeto_mem(APDS9930_ADDR, reg, bytes([val]))

def read_als():
    return int.from_bytes(i2c.readfrom_mem(APDS9930_ADDR, ALS_DATA0, 2), 'little')

def init_als():
    try:
        write_register(ENABLE, 0x03)
        print("✅ ALS enabled")
    except:
        print("⚠️ ALS init failed")

# === KALMAN FILTER ===
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

kf1, kf2 = KalmanFilter(), KalmanFilter()
SLOPE = 0.000011
DEADBAND = 0.001
tare_offset = 0.0
use_tare = True
scale_on = True
als_triggered = False

touch1_prev = False
touch2_prev = False
touch3_prev = False
frozen_weight = None

# === INIT ===
init_als()
tare_offset = ((adc1.read_u16() + adc2.read_u16()) / 2) / 65535 * 3.3
kf1.X = tare_offset
kf2.X = tare_offset

# === WIFI CONNECT ===
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("📶 Connecting...")
wlan.connect(SSID, PASSWORD)
while not wlan.isconnected(): time.sleep(0.5)
ip = wlan.ifconfig()[0]
print(f"✅ Connected: {ip}")
print(f"🌐 Server: http://{ip}:5000/weight")

# === SOCKET SERVER ===
addr = socket.getaddrinfo('0.0.0.0', 5000)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)

# === MAIN LOOP ===
while True:
    light = read_als()
    if light < THRESHOLD and not als_triggered:
        scale_on = not scale_on
        led.value(scale_on)
        print("🔄 Scale turned", "ON ✅" if scale_on else "OFF ❌")
        als_triggered = True
        time.sleep(0.5)
    elif light >= THRESHOLD:
        als_triggered = False

    try:
        cl, addr = s.accept()
        request = cl.recv(1024)
        if b"/weight" in request:
            r1 = adc1.read_u16() / 65535 * 3.3
            r2 = adc2.read_u16() / 65535 * 3.3
            f1 = kf1.update(r1)
            f2 = kf2.update(r2)
            avg = (f1 + f2) / 2

            t1 = touch1.value()
            t2 = touch2.value()
            t3 = touch3.value()

            if t1 and not touch1_prev:
                tare_offset = avg
                use_tare = True
                kf1.X = avg
                kf2.X = avg
                kf1.P, kf2.P = 1.0, 1.0
                print("🟢 TARE SET to {:.3f} V".format(tare_offset))

            if t2 and not touch2_prev:
                delta_v = tare_offset - avg if use_tare else avg
                if abs(delta_v) < DEADBAND:
                    delta_v = 0.0
                frozen_weight = max(0.0, delta_v / SLOPE)
                print("📋 WEIGHT FROZEN: {:.1f}g".format(frozen_weight))

            if t3 and not touch3_prev:
                frozen_weight = None
                print("🔄 FROZEN WEIGHT RESET")

            touch1_prev = t1
            touch2_prev = t2
            touch3_prev = t3

            if scale_on and use_tare:
                delta_v = tare_offset - avg
                if abs(delta_v) < DEADBAND:
                    delta_v = 0.0
                weight = max(0.0, delta_v / SLOPE)
            else:
                weight = 0.0

            print("🧪 Weight: {:.1f}g (Live), Frozen: {}".format(weight, frozen_weight))
            print("🧪 AVG = {:.3f} V | TARE = {:.3f} V | ΔV = {:.3f} | Weight = {:.1f}g".format(avg, tare_offset, tare_offset - avg, weight))
            response = {
                "weight": round(weight, 2),
                "frozen_weight": round(frozen_weight, 2) if frozen_weight is not None else None,
                "touch1": bool(t1),
                "touch2": bool(t2),
                "touch3": bool(t3),
                "scale_on": scale_on
            }

            body = json.dumps(response)
            cl.send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\n\r\n" + body)
        else:
            cl.send("HTTP/1.0 404 Not Found\r\n\r\n")
        cl.close()

    except Exception as e:
        print("⚠️ Error:", e)
