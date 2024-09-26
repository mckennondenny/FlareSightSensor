from machine import ADC, Pin
import time
import utime
import math

# Set up the analog pin for the smoke sensor
smoke_sensor = ADC(Pin(26))  # Use ADC0 (Pin 26)

# Get the current timestamp and create a filename
# current_time = utime.localtime()
# filename = f"sensor_log_{current_time[0]}_{current_time[1]}_{current_time[2]}_{current_time[3]}_{current_time[4]}_{current_time[5]}.txt"

# Function to read smoke sensor value
def read_smoke_level():
    raw_value = smoke_sensor.read_u16()
    voltage = raw_value * (3.3 / 65535)
    return voltage

# Open the file and log readings
with open(filename, 'w') as f:
    while True:
        smoke_v_level = read_smoke_level()
        smoke_ppm = round(81.7*math.exp(1.23*v_level))
        print(f"Smoke: {smoke_ppm} ppm")
        # f.write(log_entry)
        time.sleep(1)
