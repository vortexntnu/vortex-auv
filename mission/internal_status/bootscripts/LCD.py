#!/usr/bin/python3
import time
import subprocess
import smbus

import sys
import os
import readline
import re

from RPLCD.i2c import CharLCD
from MCP342x import MCP342x

# Raspberry Pi pin configuration:
RST = None  # on the PiOLED this pin isnt used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

# Beaglebone Black pin configuration:
# RST = 'P9_12'
# Note the following are only used with SPI:
# DC = 'P9_15'
# SPI_PORT = 1
# SPI_DEVICE = 0

IP_prev = "No IP"
IP_filt = ""

# Initialize LCD

lcd = CharLCD(i2c_expander='PCF8574',
              address=0x27,
              port=1,
              cols=16,
              rows=2,
              dotsize=8,
              charmap='A02',
              auto_linebreaks=True,
              backlight_enabled=True)

# Parameters
# to read voltage and current from ADC on PDB through I2C
i2c_adress = 0x69

# init of I2C bus communication
bus = smbus.SMBus(1)
channel_voltage = MCP342x(bus, i2c_adress, channel=0, resolution=18)  # voltage
channel_current = MCP342x(bus, i2c_adress, channel=1, resolution=18)  # current
time.sleep(1)

# Convertion ratios taken from PSM datasheet at: https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
psm_to_battery_voltage = 11.0  # V/V
psm_to_battery_current_scale_factor = 37.8788  # A/V
psm_to_battery_current_offset = 0.330  # V

xavier_IP = ""
system_voltage = 0
system_current = 0

func = 1

def read_PSM_voltage():
    # Sometimes an I/O timeout or error happens, it will run again when the error disappears
    try:
        voltage = channel_voltage.convert_and_read() * psm_to_battery_voltage

    except IOError:
        voltage = -1

    return voltage


def read_PSM_current():
    try:
        current = (channel_current.convert_and_read() -
                   psm_to_battery_current_offset
                   ) * psm_to_battery_current_scale_factor

    except IOError:
        current = -1

    return current


while True:
    # Get IP
    cmd = "hostname -I | cut -d' ' -f1"

    IP_bytes = subprocess.check_output(cmd, shell=True)
    IP_str = IP_bytes.decode("utf-8")

    # Display IP
    lcd.clear()
    lcd.write_string("IP:" + "\r\n")
    lcd.write_string(IP_str)
    time.sleep(5)
    
    # Display Voltage/Current
    for i in range(50):
        system_voltage = round(read_PSM_voltage(), 2)
        system_current = round(read_PSM_current(), 2)
        lcd.clear()
        lcd.write_string("V:" + str(system_voltage) + "\r\n")
        lcd.write_string("A:" + str(system_current))
        time.sleep(0.2)
