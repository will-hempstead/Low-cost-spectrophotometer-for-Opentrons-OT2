import sys
from time import sleep_ms
from machine import I2C, Pin
from as7341 import *
import select

# ----------------- Hardware setup -----------------
led = Pin(3, Pin.OUT)

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
sensor = AS7341(i2c)
if not sensor.isconnected():
    print("ERROR:AS7341_NOT_FOUND")
    sys.exit(1)

sensor.set_measure_mode(AS7341_MODE_SPM)
sensor.set_atime(100)
sensor.set_astep(2000)
sensor.set_again(4)

CHANNELS = ["f1","f2","f3","f4","f5","f6","f7","f8","clr","nir"]

# ----------------- Sensor functions -----------------
def measurement():
    sensor.start_measure("F1F4CN")
    f1, f2, f3, f4, clr, nir = sensor.get_spectral_data()
    sensor.start_measure("F5F8CN")
    f5, f6, f7, f8, clr, nir = sensor.get_spectral_data()
    sleep_ms(50)
    return [f1, f2, f3, f4, f5, f6, f7, f8, clr, nir]

def mean_of_replicates(n=5, led_current=20, settle_ms=100, gap_ms=100):
    sensor.set_led_current(led_current)
    sleep_ms(settle_ms)

    sums = [0] * len(CHANNELS)
    for i in range(n):
        vals = measurement()
        for j in range(len(sums)):
            sums[j] += vals[j]
        sleep_ms(gap_ms)

    sensor.set_led_current(0)
    led.off()
    return [s // n for s in sums]

# ----------------- Serial listener -----------------
def main():
    sleep_ms(5000)
    
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    
    print("READY")
    
    while True:
        try:
            events = poll.poll(1000)
            if events:
                line = sys.stdin.readline().strip()
                if line == "MEASURE":
                    led.on()
                    vals = mean_of_replicates(n=5, led_current=20,
                                               settle_ms=150, gap_ms=120)
                    print("DATA:" + ",".join(str(v) for v in vals))
                    led.off()
        except Exception as e:
            print("ERROR:" + str(e))
            led.off()

main()