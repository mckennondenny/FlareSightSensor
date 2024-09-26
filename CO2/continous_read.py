from machine import I2C, Pin
import time
import sys  # Import sys for sys.exit()

# Define I2C pins
I2C_SCL = Pin(1)  # Adjust if necessary
I2C_SDA = Pin(0)  # Adjust if necessary

# Initialize I2C bus 0 at 100 kHz frequency
i2c = I2C(0, scl=I2C_SCL, sda=I2C_SDA, freq=100000)

# Sensor I2C address and registers
SENSOR_ADDR = 0x28
MEASUREMENT_CMD_REGISTER = 0x00
STATUS_REGISTER = 0x01
CO2_DATA_HIGH = 0x02
CO2_DATA_LOW = 0x03
DATA_READY_FLAG = 0x08  # Bit 3
ERROR_FLAG = 0x02       # Bit 1
MEASURE_COMMAND = 0x01  # Start measurement
SENS_STS_REGISTER = 0x73
ERROR_CODE_REGISTER = 0xE0

def read_sensor_status():
    try:
        status = i2c.readfrom_mem(SENSOR_ADDR, SENS_STS_REGISTER, 1)
        return status[0]
    except OSError as e:
        print(f"I2C read error: {e}")
        return None

def is_sensor_ready():
    status = read_sensor_status()
    if status is None:
        return False
    sen_rdy = (status >> 7) & 0x01
    return sen_rdy == 1

def read_status_register():
    try:
        status = i2c.readfrom_mem(SENSOR_ADDR, STATUS_REGISTER, 1)
        status_value = status[0]
        print(f"Status register value: {status_value:08b}")
        return status_value
    except OSError as e:
        print(f"I2C read error: {e}")
        return None

def start_measurement():
    try:
        i2c.writeto_mem(SENSOR_ADDR, MEASUREMENT_CMD_REGISTER, bytes([MEASURE_COMMAND]))
    except OSError as e:
        print(f"I2C write error: {e}")

def wait_for_data_ready(timeout=10):
    start_time = time.time()
    while time.time() - start_time < timeout:
        status_value = read_status_register()
        if status_value is None:
            return False
        if status_value & ERROR_FLAG:
            print("Sensor reported an error.")
            read_error_code()
            return False
        if status_value & DATA_READY_FLAG:
            return True
        time.sleep(0.5)
    return False

def read_error_code():
    try:
        error_code = i2c.readfrom_mem(SENSOR_ADDR, ERROR_CODE_REGISTER, 1)[0]
        print(f"Error code: {error_code:02X}")
        return error_code
    except OSError as e:
        print(f"I2C read error: {e}")
        return None

def read_co2_data():
    try:
        co2_high = i2c.readfrom_mem(SENSOR_ADDR, CO2_DATA_HIGH, 1)[0]
        co2_low = i2c.readfrom_mem(SENSOR_ADDR, CO2_DATA_LOW, 1)[0]
        co2_concentration = (co2_high << 8) | co2_low
        return co2_concentration
    except OSError as e:
        print(f"I2C read error: {e}")
        return None

def read_co2_concentration():
    print("Status before starting measurement:")
    status_value = read_status_register()
    if status_value is None:
        return None
    start_measurement()
    if wait_for_data_ready():
        co2_concentration = read_co2_data()
        return co2_concentration
    else:
        print("Data not ready or measurement timeout.")
        return None

# Wait for sensor to initialize
print("Waiting for sensor to initialize...")

# Poll the SEN_RDY bit until the sensor is ready or a timeout occurs
start_time = time.time()
timeout = 60  # Wait up to 60 seconds for the sensor to become ready

while time.time() - start_time < timeout:
    if is_sensor_ready():
        print("Sensor is ready.")
        break
    else:
        print("Sensor not ready, waiting...")
        time.sleep(1)
else:
    print("Sensor did not become ready within timeout period.")
    sys.exit(1)  # Exit the program

# Main loop
while True:
    co2 = read_co2_concentration()
    if co2 is not None:
        print("CO₂ Concentration:", co2, "ppm")
    else:
        print("Failed to read CO₂ concentration.")
    time.sleep(1)