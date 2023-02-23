#!/usr/bin/python3
import time
import smbus
import subprocess
import readline
import re
import sys
import os

from telnetlib import IP

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

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

# 128x32 display with hardware I2C:
disp = 0
while disp == 0:
    try:
        disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1)
    except:
        print("Could not establish disp")

# Initialize library.
disp.begin()

disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.

width = disp.width
height = disp.height
image = Image.new("1", (width, height))

# Get drawing object to draw on image.

draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.

draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.

padding = -2
top = padding
bottom = height - padding

# Move left to right keeping track of the current x position for drawing shapes.
x = 0


# Load default font.
font = ImageFont.load_default()


IP_prev = "No IP"
IP_filt = ""

# Parameters
# to read voltage and current from ADC on PDB through I2C
i2c_adress = 0x69

# init of I2C bus communication
bus = smbus.SMBus(1)
channel_voltage = MCP342x(
    bus, i2c_adress, channel=0, resolution=18
)  # voltage
channel_current = MCP342x(
    bus, i2c_adress, channel=1, resolution=18
)  # current
time.sleep(1)

# Convertion ratios taken from PSM datasheet at: https://bluerobotics.com/store/comm-control-power/control/psm-asm-r2-rp/
psm_to_battery_voltage = 11.0  # V/V
psm_to_battery_current_scale_factor = 37.8788  # A/V
psm_to_battery_current_offset = 0.330  # V

xavier_IP = ""
system_voltage = 0
system_current = 0

func = 1


def func_check(func):
    if func == 1:
        draw.text((x, top + 24), "Wellness: Phase 1 ", font=font, fill=255)
        return 0
    else:
        draw.text((x, top + 24), "Wellness: Phase 2", font=font, fill=255)
        return 1


def read_PSM_voltage():
    # Sometimes an I/O timeout or error happens, it will run again when the error disappears
    try:
        voltage = channel_voltage.convert_and_read() * psm_to_battery_voltage

    except IOError:
        voltage = 0

    return voltage


def read_PSM_current():
    try:
        current = (
            channel_current.convert_and_read() - psm_to_battery_current_offset
        ) * psm_to_battery_current_scale_factor

    except IOError:
        current = 0

    return current


while True:
    cmd = "hostname -I | cut -d' ' -f1"

    # Draw a black filled box to clear the image.

    IP_bytes = subprocess.check_output(cmd, shell=True)
    IP_str = IP_bytes.decode("utf-8")

    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    # with open("xavier_IP.txt", "r") as f:
    #    xavier_IP = f.readlines()

    system_voltage = round(read_PSM_voltage(), 2)
    system_current = round(read_PSM_current(), 2)

    func = func_check(func)

    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    # Write two lines of text.

    draw.text((x, top), "IP: " + IP_str, font=font, fill=255)
    draw.text((x, top + 8),  "Volt: " + str(system_voltage), font=font, fill=255)
    draw.text((x, top + 16), "Amp:  " + str(system_current), font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(1)
