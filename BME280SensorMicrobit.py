# Bosch BME280 Pressure, Temperature and Humidity Sensor
# Language: BBC Microbit MicroPython
# Author: Terry Morris,  based upon but substantially modified Bosch sample code BME280.c
# (C) 2015 - 2016 Bosch Sensortec GmbH and BOSCHBME280PythonDriver(Adafruit).py
# (c) Adafruit Industries (Tony DiCola)
# Communications: I2C-default address 0x77
# Mode: normal (continuous conversion)
# Read Temperature, Pressure and Humidity to maximum precision, standy 125ms
# No IRR filter
# Warning: this code demands memory allocation close to the Microbit's limits!

from microbit import i2c, running_time

# BME 280 system addresses and values
# Registers
IDRegister  = 0xD0          # ID holds 0x60, always readable
RESET       = 0xE0          # Reset only executed by writing 0xB6
CTRL_HUM    = 0xF2          # Selects humidity measurement and oversampling
STATUS      = 0xF3          # Indicates data availability
CTRL_MEAS   = 0xF4          # Temperature, pressure oversampling, selects normal, forced or sleep mode
CONFIG      = 0xF5          # Controls normal mode standby time, IRR filter, SPI mode

t_fine      = 0

# Data structures
ConfigurationData           = bytearray(6)

CalData00_25                = bytearray(25)
CalData00_25BaseAddress     = bytearray(1)
CalData00_25BaseAddress[0]  = 0x88

CalData26_41                = bytearray(7)
CalData26_41BaseAddress     = bytearray(1)
CalData26_41BaseAddress[0]  = 0xE1

RawSensorData               = bytearray(8)
RawSensorDataBaseAddress    = bytearray(1)
RawSensorDataBaseAddress[0] = 0xF7

# Functions
# BuildS16: construct signed integer from 2-byte data
def BuildS16(msb, lsb):
    sval = ((msb << 8) | lsb)
    if sval > 32767:  # Largest positive value: 2**15 - 1
        sval -= 65536
    return sval
       
# BuildU16: construct unsigned integer from 2-byte data
def BuildU16(msb, lsb):
    return ((msb << 8) |lsb)

# BuildS8: construct signed integer from a single byte
def BuildS8(b):
    if b > 127:
        return (b-256)
    else:
        return b
        
# Calculate Temperature in degrees Celsius
def CalculateTemperature(TRAW):   
    global t_fine
    Traw = float(TRAW)
    v1 = (Traw/ 16384.0 - float(dig_T1) / 1024.0) * float(dig_T2)
    v2 = ((Traw / 131072.0 - float(dig_T1) / 8192.0) * (
    Traw / 131072.0 - float(dig_T1) / 8192.0)) * float(dig_T3)
    t_fine = int(v1 + v2)
    T = (v1 + v2) / 5120.0
    return T

# Calculate pressure in Pascals
def CalculatePressure(PRAW):
    Praw = float(PRAW)
    v1 = float(t_fine) / 2.0 - 64000.0
    v2 = v1 * v1 * float(dig_P6) / 32768.0
    v2 = v2 + v1 * float(dig_P5) * 2.0
    v2 = v2 / 4.0 + float(dig_P4) * 65536.0
    v1 = (float(dig_P3) * v1 * v1 / 524288.0 + float(dig_P2) * v1) / 524288.0
    v1 = (1.0 + v1 / 32768.0) * float(dig_P1)
    if v1 == 0:
        return 0
    p  = 1048576.0 - Praw
    p  = ((p - v2 / 4096.0) * 6250.0) / v1
    v1 = float(dig_P9) * p * p / 2147483648.0
    v2 = p * float(dig_P8) / 32768.0
    p  = p + (v1 + v2 + float(dig_P7)) / 16.0
    return p
    
# Calculate relative humidity
def CalculateHumidity(HRAW):
    global t_fine
    Hraw = float(HRAW)
    h = float(t_fine) - 76800.0
    h = (Hraw - (float(dig_H4) * 64.0 + float(dig_H5) / 16384.0 * h)) * (
        float(dig_H2) / 65536.0 * (1.0 + float(dig_H6) / 67108864.0 * h * (
        1.0 + float(dig_H3) / 67108864.0 * h)))
    h = h * (1.0 - float(dig_H1) * h / 524288.0)
    if h > 100:
        h = 100
    elif h < 0:
        h = 0
    return h

# Discover if the sensor is there
IDAddress     = bytearray(1)
IDAddress[0]  = IDRegister
i2c.write(0x77, IDAddress, repeat = False)
id = i2c.read(0x77, 1, repeat = False)
print(id)

# Read calibration data
i2c.write(0x77, CalData00_25BaseAddress, repeat = False)    # Send base address
CalData00_25 = i2c.read(0x77, 25, repeat = False)

i2c.write(0x77, CalData26_41BaseAddress, repeat = False)    # Send base address
CalData26_41 = i2c.read(0x77, 7, repeat = False)

#Assign calibration data variables
dig_T1 = BuildU16(CalData00_25[1], CalData00_25[0])     # unsigned short
dig_T2 = BuildS16(CalData00_25[3], CalData00_25[2])     # signed short
dig_T3 = BuildS16(CalData00_25[5], CalData00_25[4])     # signed short
dig_P1 = BuildU16(CalData00_25[7], CalData00_25[6])     # unsigned short
dig_P2 = BuildS16(CalData00_25[9], CalData00_25[8])     # signed short
dig_P3 = BuildS16(CalData00_25[11], CalData00_25[10])   # signed short
dig_P4 = BuildS16(CalData00_25[13], CalData00_25[12])   # signed short
dig_P5 = BuildS16(CalData00_25[15], CalData00_25[14])   # signed short
dig_P6 = BuildS16(CalData00_25[17], CalData00_25[16])   # signed short
dig_P7 = BuildS16(CalData00_25[19], CalData00_25[18])   # signed short
dig_P8 = BuildS16(CalData00_25[21], CalData00_25[20])   # signed short
dig_P9 = BuildS16(CalData00_25[23], CalData00_25[22])   # signed short
dig_H1 = CalData00_25[24]                               # unsigned char

dig_H2 = BuildS16(CalData26_41[1],CalData26_41[0])
dig_H3 = CalData26_41[2]
dig_H4 = (BuildS8(CalData26_41[3]) << 4) | (CalData26_41[4] & 0x0F)         # signed short presented in 12 bits
dig_H5 = (BuildS8(CalData26_41[5]) << 4) | ((CalData26_41[4] >> 4) & 0x0F)  # signed short presented in 12 bits
dig_H6 = BuildS8(CalData26_41[6])                       # signed char

# Write configuration data
# Load array with register address - value pairs
ConfigurationData[0] = CTRL_HUM         # Register address
ConfigurationData[1] = 0b00000101       # Hunidity sampling on: x16
ConfigurationData[2] = CTRL_MEAS        # Register address
ConfigurationData[3] = 0b10110111       # Temperature and pressure x16, normal mode
ConfigurationData[4] = CONFIG           # Register address
ConfigurationData[5] = 0b01000000       # Normal mode standby 125ms, IRR off, SPI irrelevant
i2c.write(0x77,ConfigurationData, repeat=False)

interval = running_time() + 500
for i in range(1,100):
    i2c.write(0x77, RawSensorDataBaseAddress, repeat = False)
    RawSensorData = i2c.read(0x77, 8, repeat = False)
    PRAW = ((RawSensorData[0] << 16) | (RawSensorData[1] << 8) | RawSensorData[2]) >> 4
    TRAW = ((RawSensorData[3] << 16) | (RawSensorData[4] << 8) | RawSensorData[5]) >> 4
    HRAW = (RawSensorData[6] << 8)   | RawSensorData[7]

    print("T = {0:6.2f}C  p = {1:5.1f}kPa  H = {2:5.1f}".format(
        CalculateTemperature(TRAW),
        CalculatePressure(PRAW)/1000,
        CalculateHumidity(HRAW)))
    
    while(running_time() < interval):
        pass
    interval = running_time() + 500
