#!/usr/bin/python3
import time
import subprocess
import smbus
import board

import sys
import os
import readline
import re

from RPLCD.i2c import CharLCD
from MCP342x import MCP342x

import adafruit_mprls

# Variables for LCD Screen
IP_prev = "No IP"
IP_filt = ""

# Initialize LCD
lcd = CharLCD(
    i2c_expander="PCF8574",
    address=0x27,
    port=1,
    cols=16,
    rows=2,
    dotsize=8,
    charmap="A02",
    auto_linebreaks=True,
    backlight_enabled=True,
)

# Parameters
i2c_adress_PSM = 0x69  # Reads voltage and current from ADC on PDB through I2C
i2c_adress_MPRLS = 0x18  # Reads pressure from MPRLS Adafruit sensor

# init of I2C bus communication
i2c_bus = smbus.SMBus(1)
channel_voltage = MCP342x(i2c_bus, i2c_adress_PSM, channel=0, resolution=18)  # voltage
channel_current = MCP342x(i2c_bus, i2c_adress_PSM, channel=1, resolution=18)  # current
channel_pressure = adafruit_mprls.MPRLS(
    board.I2C(),
    addr=i2c_adress_MPRLS,
    reset_pin=None,
    eoc_pin=None,
    psi_min=0,
    psi_max=25,
)  # Pressure
time.sleep(1)

# Convertion ratios taken from PSM datasheet at: https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
psm_to_battery_voltage = 11.0  # V/V
psm_to_battery_current_scale_factor = 37.8788  # A/V
psm_to_battery_current_offset = 0.330  # V

system_voltage = 0
system_current = 0


def read_PSM_voltage():
    # Sometimes an I/O timeout or error happens, it will run again when the error disappears
    try:
        voltage = channel_voltage.convert_and_read() * psm_to_battery_voltage

    except IOError:
        voltage = -1

    return voltage


def read_PSM_current():
    try:
        current = (
            channel_current.convert_and_read() - psm_to_battery_current_offset
        ) * psm_to_battery_current_scale_factor

    except IOError:
        current = -1

    return current


def read_internal_pressure():
    try:
        pressure = channel_pressure.pressure
    except:
        pressure = -1
    return pressure


while True:
    # Get IP
    cmd = "hostname -I | cut -d' ' -f1"

    IP_bytes = subprocess.check_output(cmd, shell=True)
    IP_str = IP_bytes.decode("utf-8")

    # Display IP [ca 5 s]
    lcd.clear()
    lcd.write_string("IP:" + "\r\n")
    lcd.write_string(IP_str)
    time.sleep(5)

    # Display pressure [ca 5 s]
    for i in range(25):
        system_pressure = round(read_internal_pressure(), 2)
        lcd.clear()
        lcd.write_string("hPa:" + str(system_pressure))
        time.sleep(0.2)

    # Display Voltage/Current [ca 10 s]
    for i in range(40):
        system_voltage = round(read_PSM_voltage(), 2)
        system_current = round(read_PSM_current(), 2)
        lcd.clear()
        lcd.write_string("V:" + str(system_voltage) + "\r\n")
        lcd.write_string("A:" + str(system_current))
        time.sleep(0.2)
