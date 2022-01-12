import time
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess

disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1) #pins 27,28 equal i2c bus 1

disp.begin()

#Clear display
disp.clear()
disp.display()

# Create blank image
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

draw = ImageDraw.Draw(image)

draw.rectangle((0,0,width,height), outline=0, fill=0)

padding = -2
top = padding
bottom = height-padding

x = 0

# Load font
font = ImageFont.truetype('ScaledFont.ttf', 16)

print("Displaying IP")

while True:
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)

    # Shell scripts for getting IP
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True )
    #Split IP-adress-string into 2 to print on seperate lines
    s = str(IP.decode('utf-8'))
    IP1 = s[:8]
    IP2 = s[len(s)-7:]

    draw.text((x, top), "IP: " + IP1,  font=font, fill=255)
    draw.text((x + 28, top + 16), IP2, font=font, fill = 255)
    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(.1)
