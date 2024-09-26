from machine import I2C, Pin
import time

# Define I2C pins
I2C_SCL = Pin(1)
I2C_SDA = Pin(0)

#initialize I2C (100kHz frequency)
i2c = I2C(0, scl=I2C_SCL, sda=I2C_SDA, freq = 100000)

# scanning (to confirm connection
devices = i2c.scan()
if devices:
    print("I2C devices found:", [hex(device) for device in devices])
else:
    print("No I2C devices found.")
