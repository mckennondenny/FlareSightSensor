from machine import ADC, Pin, I2C
import time
import utime
import math
import sys
import network
import ubinascii
from umqtt.simple import MQTTClient

# Define I2C pins
I2C_SCL = Pin(1)
I2C_SDA = Pin(0)

# Set up the analog pin for the smoke sensor
smoke_sensor = ADC(Pin(26))  # Use ADC0 (Pin 26)

# Timing Variables
PERIOD = 0x0A 

# Initialize I2C bus 0 at 100 kHz frequency
i2c = I2C(0, scl=I2C_SCL, sda=I2C_SDA, freq=400000)

# Sensor I2C address and registers
SENSOR_ADDR = 0x28

# Register addresses
SENS_RST = 0x10
MEAS_CFG_REGISTER = 0x04
MEAS_STS_REGISTER = 0x07
MEAS_RATE_L_REGISTER = 0x03
MEAS_RATE_H_REGISTER = 0x02
SENS_STS_REGISTER = 0x01
INT_CFG_REGISTER = 0x08
CO2_DATA_HIGH = 0x05
CO2_DATA_LOW = 0x06
ERROR_CODE_REGISTER = 0xE0

# Commands and flags
EARLY_START_INT =  0x18
RESET_COMMAND = 0xA3
START_MEASUREMENT_COMMAND = 0x01
SEN_RDY_BIT = 0x80  # Bit 7 in SENS_STS
DATA_READY_FLAG = 0x10  # Bit 4 in MEAS_STS
BUSY_FLAG = 0x04        # Bit 2 in MEAS_STS
ERROR_FLAG = 0x02       # Bit 1 in MEAS_STS
IDLE_MODE = 0x00
CONT_MODE = 0x02
SING_MODE = 0x01

# mosquitto_sub -h 192.168.1.92 -t "sensor/#"
# mosquitto_sub -h 192.168.1.92 -t "sensor/co2"
# mosquitto_sub -h 192.168.1.92 -t "sensor/smoke"

# Wi-Fi Configuration
SSID = 'HamptonA'
PASSWORD = 'Stroud2Dell'
MQTT_BROKER = '192.168.1.92'  
MQTT_PORT = 1883
CLIENT_ID = ubinascii.hexlify(machine.unique_id())
MQTT_TOPIC_CO2 = b"sensor/co2"
MQTT_TOPIC_SMOKE = b"sensor/smoke"

# Wi-Fi Connection Function
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    max_wait = 15
    while max_wait > 0:
        if wlan.isconnected():
            print('Connected to Wi-Fi')
            print(wlan.ifconfig())
            break
        max_wait -= 1
        print('Waiting for connection...')
        time.sleep(1)
    else:
        print('Could not connect to Wi-Fi')
        sys.exit()

# MQTT Publish Function
def publish(topic, msg):
    try:
        client.publish(topic, msg)
        print(f'Published {msg} to {topic}')
    except Exception as e:
        print(f'Failed to publish message: {e}')

# Initialize MQTT Client
def init_mqtt():
    global client
    try:
        client = MQTTClient(CLIENT_ID, MQTT_BROKER, MQTT_PORT)
        client.connect()
        print('Connected to MQTT Broker')
    except Exception as e:
        print(f'Failed to connect to MQTT Broker: {e}')
        sys.exit()

# Function to read smoke sensor value
def read_smoke_level():
    raw_value = smoke_sensor.read_u16()
    voltage = raw_value * (3.3 / 65535)
    return voltage

def reset_sensor():
    try:
        i2c.writeto_mem(SENSOR_ADDR, SENS_RST, bytes([RESET_COMMAND]))
        time.sleep(1)  # Wait 10ms after reset   
        print("Sensor reset.")
        
    except OSError as e:
        print(f"I2C write error during sensor reset: {e}")

def read_sensor_status():
    try:
        status = i2c.readfrom_mem(SENSOR_ADDR, SENS_STS_REGISTER, 1)
        status_value = status[0]
        # print(f"SENS_STS register value: {status_value:08b}")
        return status_value
    except OSError as e:
        print(f"I2C read error: {e}")
        return None

def is_sensor_ready():
    status_value = read_sensor_status()
    if status_value is None:
        return False
    sen_rdy = status_value & SEN_RDY_BIT
    return sen_rdy == SEN_RDY_BIT

# Function to check if data is ready
def is_data_ready():
    try:
        time.sleep(0.1)
        sen = i2c.readfrom_mem(SENSOR_ADDR, SENS_STS_REGISTER, 1)[0]
        # print(f"SENS_STS register value: {sen:08b}")
        status = i2c.readfrom_mem(SENSOR_ADDR, MEAS_STS_REGISTER, 1)
        time.sleep(0.1)
        status_value = status[0]
        # print(f"MEAS_STS register value: {status_value:08b}")
        return status_value & DATA_READY_FLAG == DATA_READY_FLAG
    except OSError as e:
        print(f"DATA READY I2C read error: {e}")
        return False

def read_measurement_status():
    try:
        status = i2c.readfrom_mem(SENSOR_ADDR, MEAS_STS_REGISTER, 1)
        status_value = status[0]
        # print(f"MEAS_STS register value: {status_value:08b}")
        return status_value
    except OSError as e:
        print(f"I2C read error: {e}")
        return None

def start_measurement():
    try:
        i2c.writeto_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, bytes([START_MEASUREMENT_COMMAND]))
        print("Measurement started.")
    except OSError as e:
        print(f"I2C write error: {e}")

def wait_for_data_ready(timeout=20):
    start_time = time.time()
    while time.time() - start_time < timeout:
        status_value = read_measurement_status()
        data_ready = status_value & 0x10 
        if data_ready:
            print("Data is ready.")
            return True
        else:
            print("Measurement not active.")
        time.sleep(0.5)
    return False  # Timeout

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
    start_measurement()
    if wait_for_data_ready():
        co2_concentration = read_co2_data()
        return co2_concentration
    else:
        print("Data not ready or measurement timeout.")
        return None
    
def read_meas_cfg():
    try:
        meas_cfg = i2c.readfrom_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, 1)
        meas_cfg_value = meas_cfg[0]
        print(f"MEAS_CFG register value: {meas_cfg_value:08b}")
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
    
def read_meas_sts():
    try:
        meas_sts = i2c.readfrom_mem(SENSOR_ADDR, MEAS_STS_REGISTER, 1)
        meas_sts_value = meas_sts[0]
        print(f"MEAS_STS register value: {meas_sts_value:08b}")
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
    
def read_meas_cfg():
    try:
        meas_cfg = i2c.readfrom_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, 1)
        meas_cfg_value = meas_cfg[0]
        print(f"MEAS_CFG register value: {meas_cfg_value:08b}")
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
    
def read_sens_sts():
    try:
        sens_sts = i2c.readfrom_mem(SENSOR_ADDR, SENS_STS_REGISTER, 1)
        sens_sts_value = sens_sts[0]
        print(f"SENS_STS register value: {sens_sts_value:08b}")
    except OSError as e:
        print(f"SENS_STS read error: {e}")
        return None

def read_int_cfg():
    try:
        int_cfg = i2c.readfrom_mem(SENSOR_ADDR, INT_CFG_REGISTER, 1)
        int_cfg_value = int_cfg[0]
        print(f"INT_CFG register value: {int_cfg_value:08b}")
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
    
def read_single_shot():
    try:
        i2c.writeto_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, bytes([SING_MODE]))  
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
    
def init_cont_mode():
    try:
        i2c.writeto_mem(SENSOR_ADDR, MEAS_RATE_L_REGISTER, bytes([0x00]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, MEAS_RATE_H_REGISTER, bytes([0x00]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, MEAS_RATE_L_REGISTER, bytes([0x0A]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, bytes([CONT_MODE]))
        time.sleep(0.1)
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
    
def init_int_cfg():
    try:
        i2c.writeto_mem(SENSOR_ADDR, INT_CFG_REGISTER, bytes([EARLY_START_INT]))  
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
    
def read_meas_rate():
    low = i2c.readfrom_mem(SENSOR_ADDR, MEAS_RATE_L_REGISTER, 1)[0]
    high = i2c.readfrom_mem(SENSOR_ADDR, MEAS_RATE_H_REGISTER, 1)[0]
    
    #print(f"MEAS_RATE_L register value: {low:08b}")
    #print(f"MEAS_RATE_H register value: {low:08b}")
    
    period = (high << 8) | low
    return period
    
def init_sensor():
    init_cont_mode()
    init_int_cfg()
    
    
# Begin initialization
print("Resetting C02 sensor...")
reset_sensor()

read_meas_rate()

# Wait for sensor to initialize
print("Waiting for CO2 sensor to initialize...")
init_sensor()

start_time = time.time()
timeout = 60  # Wait up to 60 seconds for the sensor to become ready

while time.time() - start_time < timeout:
    if is_sensor_ready():
        print("C02 sensor is ready.")
        break
    else:
        time.sleep(1)
else:
    print("C02 sensor did not become ready within timeout period.")
    sys.exit(1)  # Exit the program
    

period = read_meas_rate()
print(f"The period of measurement for the C02 sensor is {period} seconds.")

# Connect to Wi-Fi
connect_wifi()

# Initialize MQTT
init_mqtt()


# Main loop to read CO2 concentration every 10 seconds
print("Starting continuous measurement of CO2 and Smoke...")
sec = 0

# ******* INSERT OFFSET TO AVOID ERROR? ************
time.sleep(1)

while True:
    # read smoke concentration roughly every second
    for i in range(0,11):
        time.sleep(1)
        smoke_v_level = read_smoke_level()
        smoke_ppm = round(81.7*math.exp(1.23*smoke_v_level))
        smoke_message = str(smoke_ppm).encode('utf-8')
        print(f"Smoke Concentration: {smoke_ppm} ppm")
        publish(MQTT_TOPIC_SMOKE, smoke_message)
        
    if is_data_ready():
        co2_concentration = read_co2_data()
        if co2_concentration is not None:
            co2_message = str(co2_concentration).encode('utf-8')
            # Publish CO2 data
            publish(MQTT_TOPIC_CO2, co2_message)
        else:
            print("Failed to read CO2 concentration.")
