from machine import I2C, Pin
import time
import sys

# Define I2C pins
I2C_SCL = Pin(1)
I2C_SDA = Pin(0)

# Timing Variables
PERIOD = 0x0A 

# Initialize I2C bus 0 at 400 kHz frequency
i2c = I2C(0, scl=I2C_SCL, sda=I2C_SDA, freq=400000)

# Sensor I2C address and registers
SENSOR_ADDR = 0x28

# Register addresses
SENS_RST_REGISTER = 0x10
MEAS_CFG_REGISTER = 0x04
MEAS_STS_REGISTER = 0x07
MEAS_RATE_L_REGISTER = 0x03
MEAS_RATE_H_REGISTER = 0x02
SENS_STS_REGISTER = 0x01
INT_CFG_REGISTER = 0x08
CO2_DATA_HIGH = 0x05
CO2_DATA_LOW = 0x06
CALIB_REF_H_REGISTER = 0x0D
CALIB_REF_L_REGISTER = 0x0E

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

# Calibration Values
cal_val_l = 0x90
cal_val_h = 0x01

# function to reset sensor
def reset_sensor():
    try:
        i2c.writeto_mem(SENSOR_ADDR, SENS_RST_REGISTER, bytes([RESET_COMMAND]))
        time.sleep(1)  # Wait 10ms after reset   
        print("Sensor reset.")
        
    except OSError as e:
        print(f"I2C write error during sensor reset: {e}")

# Function that initializes the measurement rate to 10s
def init_meas_rate_10():
    try:
        i2c.writeto_mem(SENSOR_ADDR, MEAS_RATE_L_REGISTER, bytes([0x00]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, MEAS_RATE_H_REGISTER, bytes([0x00]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, MEAS_RATE_L_REGISTER, bytes([0x0A]))
        time.sleep(0.1)
    except OSError as e:
        print(f"MEAS I2C read error: {e}")
        return None
    
# Function that initializes calibration registers so the level is 400 ppm
def init_cal_ref_400():
    try:
        i2c.writeto_mem(SENSOR_ADDR, CALIB_REF_H_REGISTER, bytes([0x00]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, CALIB_REF_L_REGISTER, bytes([0x00]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, CALIB_REF_H_REGISTER, bytes([cal_val_l]))
        time.sleep(0.1)
        i2c.writeto_mem(SENSOR_ADDR, CALIB_REF_L_REGISTER, bytes([cal_val_h]))
        time.sleep(0.1)
    except OSError as e:
        print(f"CAL I2C read error: {e}")
        return None
    
def read_cal_ref():
    cal_high = i2c.readfrom_mem(SENSOR_ADDR, CALIB_REF_H_REGISTER, 1)[0]
    cal_low = i2c.readfrom_mem(SENSOR_ADDR, CALIB_REF_L_REGISTER, 1)[0]
    cal = (cal_high << 8) | cal_low
    return cal

def read_meas_rate():
    low = i2c.readfrom_mem(SENSOR_ADDR, MEAS_RATE_L_REGISTER, 1)[0]
    high = i2c.readfrom_mem(SENSOR_ADDR, MEAS_RATE_H_REGISTER, 1)[0]    
    period = (high << 8) | low
    return period

def init_fcs_cont():
    FCS_CNT_MASK = 0x0A
    i2c.writeto_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, bytes([FCS_CNT_MASK]))
    time.sleep(0.1)
    
def read_drdy():
    try:
        sts = i2c.readfrom_mem(SENSOR_ADDR, MEAS_STS_REGISTER, 1)[0]
        return ((sts & 0x10) >> 4)
    except OSError as e:
        print(f"DRDY I2C read error: {e}")
        

def read_co2_data():
    try:
        co2_high = i2c.readfrom_mem(SENSOR_ADDR, CO2_DATA_HIGH, 1)[0]
        co2_low = i2c.readfrom_mem(SENSOR_ADDR, CO2_DATA_LOW, 1)[0]
        co2_concentration = (co2_high << 8) | co2_low
        return co2_concentration
    except OSError as e:
        print(f"C02 I2C read error: {e}")
        return None
    
def read_meas_cfg():
    try:
        meas_cfg = i2c.readfrom_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, 1)
        meas_cfg_value = meas_cfg[0]
        print(f"MEAS_CFG register value: {meas_cfg_value:08b}")
    except OSError as e:
        print(f"MEAS CFG I2C read error: {e}")
        return None
    
def read_meas_sts():
    try:
        meas_sts = i2c.readfrom_mem(SENSOR_ADDR, MEAS_STS_REGISTER, 1)
        meas_sts_value = meas_sts[0]
        print(f"MEAS_STS register value: {meas_sts_value:08b}")
    except OSError as e:
        print(f"I2C read error: {e}")
        return None
# reset sensor
reset_sensor()

print("Initializing FCS...")

# set idle mode
i2c.writeto_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, bytes([IDLE_MODE]))

# set measurement rate to 10 seconds and calibration register to 400ppm
init_meas_rate_10()

calibration_level = read_cal_ref()
measurement_rate = read_meas_rate()

print(f"PPM Ref: {calibration_level} ppm")
print(f"Measurement Period: {measurement_rate} seconds")

init_fcs_cont()
read_meas_cfg()
read_meas_sts()

for i in range(3):
    time.sleep(10.5)   
    read_drdy()
    data = read_co2_data()
    print(f"CO2 Concentration: {data} ppm")
    

# # set idle mode
# i2c.writeto_mem(SENSOR_ADDR, MEAS_CFG_REGISTER, bytes([IDLE_MODE]))   
# 
# # save calibration
# i2c.writeto_mem(SENSOR_ADDR, SENS_RST_REGISTER, bytes([0xCF]))







