from machine import ADC, Pin
import time

# Set up the analog pins for both sensors
gas_sensor_1 = ADC(Pin(26))  # First sensor on ADC0 (Pin 31, GP26)
gas_sensor_2 = ADC(Pin(27))  # Second sensor on ADC1 (Pin 32, GP27)

# Function to read gas sensor voltage
def read_gas_level(sensor):
    # Read the analog value (range 0-65535)
    raw_value = sensor.read_u16()
    
    # Convert raw value to voltage (assuming 3.3V reference)
    voltage = raw_value * (3.3 / 65535)
    
    return voltage

# Main loop to read and print values from both sensors
while True:
    gas_level_1 = read_gas_level(gas_sensor_1)
    gas_level_2 = read_gas_level(gas_sensor_2)
    
    print(f"Gas Sensor 1 Voltage: {gas_level_1:.2f} V")
    print(f"Gas Sensor 2 Voltage: {gas_level_2:.2f} V")
    
    time.sleep(1)
