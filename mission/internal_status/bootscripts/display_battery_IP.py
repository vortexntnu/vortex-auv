#!/usr/bin/python3

import readline
from telnetlib import IP
import time

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import re

import subprocess
import smbus



# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isnt used
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
bus = smbus.SMBus(1)

# Initialize library.

disp.begin()

disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.

width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.

draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.

draw.rectangle((0,0,width,height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.

padding = -2
top = padding
bottom = height-padding

# Move left to right keeping track of the current x position for drawing shapes.
x = 0


# Load default font.
font = ImageFont.load_default()


IP_prev = 'No IP'
IP_filt = ''


# ro red voltage and current from Arduino Nano through I2C
        # for code on the arduino: 
nano_addr = 12             # I2C adress of nano (setted in software!)
voltage_reg_nano = 0       # value to send to arduino to get voltage read back
current_reg_nano = 1       # to get current measurement back

    # Calibration values for converting from raw digital binary form to decimal form
    # Calibration values were manualy calibrated to +- 0.1V acuracy!
calVoltageA = 11
calVoltageB = 0.0
calCurrent = 37.8788
calCurrentOffset = 0.33


        # init of I2C bus with arduino nano conected

bus = smbus.SMBus(1)

xavier_IP = ""
system_voltage = 0
system_current = 0

func = 1
def func_check(func):
    if func == 1:
        draw.text((x, top+16),     "Wellness: Phase 1 ", font=font, fill=255)
        return 0
    else:
        draw.text((x, top+16),     "Wellness: Phase 2", font=font, fill=255)
        return 1


def read_voltage():
        # Sometimes an I/O timeout or error happens, it will run again when the error disappears
        try:
            # arduino is configure to send voltage data on "register" 0, current on 1
            # data is sent in 2 bytes, because to big for one I2C message
            voltage_msg = bus.read_i2c_block_data(nano_addr, 0, 2)

            # conversion to get real voltage
            # measurement up to 1023, so to big for 7bit I2C messages. Sends MSB first, then LSB, then remap to 0-5V
            x = (((voltage_msg[0]&0x7) << 7) + voltage_msg[1]) * 5/1023.0
            system_voltage = x * calVoltageA + calVoltageB


            #####################################################################################################################
            voltage_msg = bus.read_i2c_block_data(nano_addr, 0, 2)
            # conversion to get real voltage
            # measurement up to 1023, so to big for 7bit I2C messages. Sends MSB first, then LSB
            system_voltage = float((((voltage_msg[0]&0x7) << 7) + voltage_msg[1])) * calVoltageA + calVoltageB
            #####################################################################################################################
            return system_voltage

        except IOError:
            print("Bus IOerror")


def read_current():
    try:
        current_msg = bus.read_i2c_block_data(nano_addr, 1, 2)

        # conversion to get real voltage
        x = float((((current_msg[0]&0x7) << 7) + current_msg[1])) * 5 / 1023.0
        system_current = (x - calCurrentOffset) * calCurrent
            #rospy.loginfo(f"Current : {system_current}A")
    except IOError:
        print("BUS error")

while True:
    

    cmd = "hostname -I | cut -d\' \' -f1"
    
    # Draw a black filled box to clear the image.
    
    IP_bytes = subprocess.check_output(cmd, shell = True )
    IP_str = IP_bytes.decode('utf-8')


    draw.rectangle((0,0,width,height), outline=0, fill=0)

    

    #with open("xavier_IP.txt", "r") as f:
    #    xavier_IP = f.readlines()

    

    system_voltage = read_voltage()
    #read_current()
    
    func = func_check(func)

    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    # Write two lines of text.

    draw.text((x, top),       "IP: " + IP_str,  font=font, fill=255)
    draw.text((x, top+8),     "Voltage: "+ str(system_voltage), font=font, fill=255)
    #draw.text((x, top+16),     "Xavier: "+ xavier_IP, font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(1)
    
