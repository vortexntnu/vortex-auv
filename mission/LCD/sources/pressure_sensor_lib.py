#!/usr/bin/python3
# Python Libraries
import time

# Pressure sensor Libraries
import board
import adafruit_mprls  # ! NOTE: sudo pip3 install adafruit-circuitpython-mprls


class PressureSensor:
    def __init__(self):
        # Pressure Sensor Setup
        i2c_adress_MPRLS = 0x18  # Reads pressure from MPRLS Adafruit sensor
        self.channel_pressure = None

        try:
            I2CBuss = board.I2C()
            self.channel_pressure = adafruit_mprls.MPRLS(
                i2c_bus=I2CBuss,
                addr=i2c_adress_MPRLS,
                reset_pin=None,
                eoc_pin=None,
                psi_min=0,
                psi_max=25,
            )
        except Exception as error:
            print(f"ERROR: Couldn't connect to Pressure Sensor: {error}")

        time.sleep(1)

    def get_pressure(self):
        try:
            pressure = self.channel_pressure.pressure
            return pressure
        except Exception as error:
            print(f"ERROR: Failed to get pressure: {error}")
            return 0.0
