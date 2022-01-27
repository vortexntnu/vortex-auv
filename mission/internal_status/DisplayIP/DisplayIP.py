import time
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess
import smbus

disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1) #pins 27,28 equal i2c bus 1

bus = smbus.SMBus(8)

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
font = ImageFont.load_default()

print("Displaying IP")

while True:
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)

    # Shell scripts for getting IP
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True )
    #Split IP-adress-string into 2 to print on seperate lines
    s = str(IP.decode('utf-8'))
    #IP1 = s[:8]
    #IP2 = s[len(s)-8:]
    xavier_mV = int(subprocess.check_output(["cat", "/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_voltage0_input"]).decode("utf-8"))
    xavier_V =str(xavier_mV / 1000.0)

    system_V = bus.read_i2c_block_data(0x6a, 0x00, 2)
    raw_adc_voltage = (system_V[0] & 0x0F) * 256 + system_V[1]
    system_V = str(raw_adc_voltage * 0.011)
    print(system_V)
    print(xavier_V)

    draw.text((x+2, top+3), "IP: " + IP,  font=font, fill=255)
    #draw.text((x + 28, top + 16), IP2, font=font, fill = 255)
    draw.text((x+2, top + 12),"Xavier: " + xavier_V + "v", font=font, fill = 255)
    draw.text((x+2, top + 20),"System: " + system_V +"v", font=font, fill = 255)

    #draw.rectangle((0,0,width,height), outline=0, fill=0)
    #draw.rectangle((10,60,30,50), outline=1, fill=0)
    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(.1)
