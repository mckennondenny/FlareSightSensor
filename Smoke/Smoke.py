from machine import ADC, Pin
import time
import utime

# Set up the analog pin for the gas sensor
gas_sensor = ADC(Pin(26))  # Use ADC0 (Pin 26)

# Get the current timestamp and create a filename
current_time = utime.localtime()
filename = f"sensor_log_{current_time[0]}_{current_time[1]}_{current_time[2]}_{current_time[3]}_{current_time[4]}_{current_time[5]}.txt"

# Function to read gas sensor value
def read_gas_level():
    raw_value = gas_sensor.read_u16()
    voltage = raw_value * (3.3 / 65535)
    return voltage

# Open the file and log readings
with open(filename, 'w') as f:
    while True:
        gas_level = read_gas_level()
        log_entry = f"Gas Sensor Voltage: {gas_level:.2f} V\n"
        print(log_entry)
        f.write(log_entry)
        time.sleep(1)
