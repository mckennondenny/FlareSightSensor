from machine import ADC, Pin, I2C
import time
import utime
import math
import sys
import network
import ntptime  # For NTP time synchronization
import ubinascii
from umqtt.simple import MQTTClient

# Wi-Fi Configuration
SSID = 'HamptonA'
PASSWORD = 'Stroud2Dell'

# MQTT Configuration
MQTT_BROKER = 'Your_Broker_IP'
MQTT_PORT = 1883
CLIENT_ID = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()

# MQTT Topics
MQTT_TOPIC_CO2 = b"sensor/co2"
MQTT_TOPIC_SMOKE = b"sensor/smoke"
MQTT_TOPIC_CO = b"sensor/co"

# Define I2C pins
I2C_SCL = Pin(1)
I2C_SDA = Pin(0)

# Initialize I2C bus 0 at 100 kHz frequency
i2c = I2C(0, scl=I2C_SCL, sda=I2C_SDA, freq=400000)

# Sensor I2C address and registers
SENSOR_ADDR = 0x28

# Set up the analog pins for the sensors
smoke_sensor = ADC(Pin(26))    # Smoke sensor on ADC0 (Pin 26)
co_sensor_adc = ADC(Pin(27))   # CO sensor on ADC1 (Pin 27)

# GPIO pin to control the heater voltage for CO sensor (via MOSFET)
heater_pin = Pin(15, Pin.OUT)  # Use Pin 15 as the digital output for heater control

# Heater cycle timings in seconds
HIGH_VOLTAGE_DURATION = 60   # 60 seconds at 5V (heating phase)
LOW_VOLTAGE_DURATION = 90    # 90 seconds at 1.4V (sensing phase)

# Initialize heater state and timer
heater_state = 'HIGH'
heater_timer_start = time.time()

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

# Get the current local time
now = time.localtime()

# Manually format the time into a string for the filename
filename = '{:04d}-{:02d}-{:02d}_{:02d}-{:02d}-{:02d}.txt'.format(
    now[0], now[1], now[2], now[3], now[4], now[5]
)

# Function to control the heater voltage
def set_heater_voltage(state):
    if state == 'HIGH':
        heater_pin.high()  # Set GPIO pin HIGH for 5V phase
        print("Heater set to HIGH voltage (5V).")
    elif state == 'LOW':
        heater_pin.low()   # Set GPIO pin LOW for 1.4V phase
        print("Heater set to LOW voltage (1.4V).")
    else:
        print("Invalid heater state.")

set_heater_voltage(heater_state)

# Function to read smoke sensor value
def read_smoke_level():
    raw_value = smoke_sensor.read_u16()
    voltage = raw_value * (3.3 / 65535)
    return voltage

# Function to read CO sensor value
def read_co_sensor():
    raw_value = co_sensor_adc.read_u16()
    voltage = raw_value * (3.3 / 65535)  # Convert ADC reading to voltage
    co_ppm = calculate_co_ppm(voltage)
    return co_ppm

# Function to calculate CO PPM using the given equation: 1.643^(1.22 * voltage)
def calculate_co_ppm(voltage):
    co_ppm = 1.643 ** (1.22 * voltage)
    return co_ppm

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

# Connect to Wi-Fi
# connect_wifi()

# Initialize MQTT
# init_mqtt()

# Function to log data to a file
def log_data_to_file(smoke, co2, co):
    try:
        with open(filename, 'a') as f:
            f.write('Smoke: '+ str(smoke) + ';' + 'CO2: '+ str(co2) + ';' + 'CO: ' + str(co2) + ';\n')
        # print(f"Data logged: {data}")
    except Exception as e:
        print(f"Failed to log data: {e}")

# ******* INSERT OFFSET TO AVOID ERROR? ************
time.sleep(0.5)

def print_readings(smoke, co2, co):
    print(f"Smoke: {smoke} ppm")
    print(f"C02: {co2} ppm")
    print(f"C0: {co} ppm")
    print()
    

smoke_reading = 0
co_reading = 0
co2_reading = 0

print(f"File name:{filename}.txt")
# Main loop to sample all sensors
while True:
    # Read smoke concentration roughly every second
    for i in range(0, 11):  # Loop runs approximately 11 seconds
        time.sleep(1.1)
        smoke_v_level = read_smoke_level()
        smoke_ppm = round(81.7 * math.exp(1.23 * smoke_v_level))
        smoke_message = f"Smoke Concentration: {smoke_ppm} ppm"
        smoke_reading = smoke_ppm
        print_readings(smoke_reading, co2_reading, co_reading)
        log_data_to_file(smoke_reading, co2_reading, co_reading)

        # CO Sensor Heater Cycle Management
        current_time = time.time()
        elapsed_time = current_time - heater_timer_start

        if heater_state == 'HIGH' and elapsed_time >= HIGH_VOLTAGE_DURATION:
            # Switch to LOW voltage phase (sensing phase)
            set_heater_voltage('LOW')
            heater_state = 'LOW'
            heater_timer_start = current_time

        elif heater_state == 'LOW' and elapsed_time >= LOW_VOLTAGE_DURATION:
            # Read CO sensor before switching back to HIGH voltage
            co_ppm = read_co_sensor()
            co_message = f"CO Concentration: {co_ppm:.2f} ppm"
            co_reading = round(co_ppm,2)
            print_readings(smoke_reading, co2_reading, co_reading)
            log_data_to_file(smoke_reading, co2_reading, co_reading)

            # Switch back to HIGH voltage phase (heating phase)
            set_heater_voltage('HIGH')
            heater_state = 'HIGH'
            heater_timer_start = current_time

    # CO₂ Sensor Sampling after smoke sensor loop (~11 seconds)
    if is_data_ready():
        co2_concentration = read_co2_data()
        if co2_concentration is not None:
            co2_message = f"CO2 Concentration: {co2_concentration} ppm"
            co2_reading = co2_concentration
            print_readings(smoke_reading, co2_reading, co_reading)
            log_data_to_file(smoke_reading, co2_reading, co_reading)
        else:
            print("Failed to read CO2 concentration.")
