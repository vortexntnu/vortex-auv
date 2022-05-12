import time
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess
import smbus

disp = Adafruit_SSD1306.SSD1306_128_64(
    rst=None, i2c_bus=1, gpio=1
)  # pins 27,28 equal i2c bus 1

bus = smbus.SMBus(8)

disp.begin()
# Clear display
disp.clear()
disp.display()

# Create blank image
width = disp.width
height = disp.height
image = Image.new("1", (width, height))

draw = ImageDraw.Draw(image)

draw.rectangle((0, 0, width, height), outline=0, fill=0)

padding = -2
top = padding
bottom = height - padding

x = 0

# Load font
font = ImageFont.load_default()

while True:
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    # Shell scripts for getting IP
    cmd = "hostname -I | cut -d' ' -f1"
    IP = subprocess.check_output(cmd, shell=True)

    xavier_mV = int(
        subprocess.check_output(
            [
                "cat",
                "/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_voltage0_input",
            ]
        ).decode("utf-8")
    )
    xavier_V = str(xavier_mV / 1000.0)

    system_V = bus.read_i2c_block_data(0x6A, 0x00, 2)
    raw_adc_voltage = (system_V[0] & 0x0F) * 256 + system_V[1]
    system_V = str(raw_adc_voltage * 0.011)

    # Script for getting SSH
    cmd = "w -h | wc -l"
    n_of_users = subprocess.check_output(cmd, shell=True)

    draw.text((x + 2, top + 3), "IP: " + IP, font=font, fill=255)
    draw.text((x + 2, top + 12), "Xavier: " + xavier_V + "v", font=font, fill=255)
    draw.text((x + 2, top + 20), "System: " + system_V + "v", font=font, fill=255)
    draw.text((x + 2, top + 40), "No. users: " + n_of_users, font=font, fill=255)

    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(0.1)
