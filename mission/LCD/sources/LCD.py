#!/usr/bin/python3
# Python Libraries
from time import sleep

# Custom Libraries
from LCD_lib import LCDScreenDriver
from IP_lib import IPDriver
from power_sense_module_lib import PowerSenseModule
from pressure_sensor_lib import PressureSensor
from temperature_sensor_lib import TemperatureSensor

# Initialize all necesarry drivers/libraries for the display and functionality
LCD = LCDScreenDriver()
IP = IPDriver()
PSM = PowerSenseModule()
Pressure = PressureSensor()
Temperature = TemperatureSensor()


# Formating function for nices LCD screen layout
def format_line(value: str, unit: str):
    spacesAvailable = 16
    valueLenght = len(value)
    unitLenght = (
        len(unit) + 1
    )  # +1 to make sure there is spacing between value and unit

    emptySpaceLenght = spacesAvailable - (valueLenght + unitLenght)
    if emptySpaceLenght < 0:
        emptySpaceLenght = 0

    formatedString = value[0 : (spacesAvailable - unitLenght)]
    formatedString += " " * emptySpaceLenght
    formatedString += " " + unit

    return formatedString


# Fancy animation at the start
LCD.fancy_animation(animation_speed=3.0)

# Display information on the LDC Screen
while True:
    # IP ----------
    timeDispaying = 5
    updatesPerSecond = 1
    for i in range(timeDispaying * updatesPerSecond):
        line1 = "IP: "
        line2 = str(IP.get_IP())
        LCD.write_to_screen(line1, line2)
        sleep(1 / updatesPerSecond)

    # Voltage and Current ----------
    timeDispaying = 5
    updatesPerSecond = 2
    for i in range(timeDispaying * updatesPerSecond):
        line1 = format_line(str(round(PSM.get_voltage(), 3)), "V")
        line2 = format_line(str(round(PSM.get_current(), 3)), "A")
        LCD.write_to_screen(line1, line2)
        sleep(1 / updatesPerSecond)

    # Pressure and Temperature ----------
    timeDispaying = 5
    updatesPerSecond = 1
    for i in range(timeDispaying * updatesPerSecond):
        line1 = format_line(str(round(Pressure.get_pressure(), 1)), "hPa")
        line2 = format_line(str(round(Temperature.get_temperature(), 1)), "*C")
        LCD.write_to_screen(line1, line2)
        sleep(1 / updatesPerSecond)
