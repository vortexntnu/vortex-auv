# import time
# import Adafruit_SSD1306
# from PIL import Image
# from PIL import ImageDraw
# from PIL import ImageFont
# import subprocess
# import smbus

# disp = Adafruit_SSD1306.SSD1306_128_64(
#     rst=None, i2c_bus=1, gpio=1
# )  # pins 27,28 equal i2c bus 1

# #bus = smbus.SMBus(8)

# disp.begin()
# # Clear display
# disp.clear()
# disp.display()

# # Create blank image
# width = disp.width
# height = disp.height
# image = Image.new("1", (width, height))

# draw = ImageDraw.Draw(image)

# draw.rectangle((0, 0, width, height), outline=0, fill=0)

# padding = -2
# top = padding
# bottom = height - padding

# x = 0

# # Load font
# font = ImageFont.load_default()

# while True:
#     # Draw a black filled box to clear the image.
#     draw.rectangle((0, 0, width, height), outline=0, fill=0)

#     # Shell scripts for getting IP
#     cmd = "hostname -I | cut -d' ' -f1"
#     IP = subprocess.check_output(cmd, shell=True)

#     #xavier_mV = int(
#      #   subprocess.check_output(
#       #      [
#        #         "cat",
#         #        "/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_voltage0_input",
#          #   ]
#        # ).decode("utf-8")
#     #)
#     #xavier_V = str(xavier_mV / 1000.0)

#     #system_V = bus.read_i2c_block_data(0x6A, 0x00, 2)
#     #raw_adc_voltage = (system_V[0] & 0x0F) * 256 + system_V[1]
#     #system_V = str(raw_adc_voltage * 0.011)

#     # Script for getting SSH
#     cmd = "w -h | wc -l"
#     n_of_users = subprocess.check_output(cmd, shell=True)

#     print(IP)

#     draw.text((x + 2, top + 3), "IP: " + IP, font=font, fill=255)
#     #draw.text((x + 2, top + 12), "Xavier: " + xavier_V + "v", font=font, fill=255)
#     #draw.text((x + 2, top + 20), "System: " + system_V + "v", font=font, fill=255)
#     #draw.text((x + 2, top + 40), "No. users: " + n_of_users, font=font, fill=255)

#     # Display image.
#     disp.image(image)
#     disp.display()
#     time.sleep(0.1)

# For use with I2C OLED screens. 
# This requires the Adafruit Circuit Python OLED library, which superceeds earlier Adafruit OLED libraries
# Install it with `pip install adafruit-circuitpython-ssd1306`

import time
from subprocess import check_output

from board import SCL, SDA
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

def get_ip():
	#cmd = "hostname -I | cut -d\' \' -f1"
	cmd = "ip -4 -o address show dev eth0 | awk '{print $4}'"
	return check_output(cmd, shell=True).decode("utf-8").strip()


# Create the I2C interface.
i2c = busio.I2C(SCL, SDA)

# Create the SSD1306 OLED class.
# The first two parameters are the pixel width and pixel height.  Change these
# to the right size for your display!
disp = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)

# Clear display.
disp.fill(0)
disp.show()

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

no_IP = True

draw.text((x, top + 0), "Starting search for WiFi", font=font, fill=255)
disp.image(image)
disp.show()
time.sleep(1)