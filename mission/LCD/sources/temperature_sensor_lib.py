#!/usr/bin/python3
"""
! NOTE: 
! For now we don't have a external sensor to measure internal temerature
! Instead we just use Internal Computer temperature sensor to gaugue temperature of teh enviroment aproximately
! In the future someone should implement a external temperture sensor for measuting a more acurate state of the temperatuer on the inside of the AUV
"""

# Python Libraries
import subprocess


class TemperatureSensor:
    def __init__(self):
        # Temperature Sensor Setup
        self.temperature_sensor_file_location = "/sys/class/thermal/thermal_zone0/temp"

    def get_temperature(self):
        try:
            # Read internal temperature on the computer
            result = subprocess.run(
                ["cat", self.temperature_sensor_file_location],
                capture_output=True,
                text=True,
                check=False,
            )

            # Decode and strip to get rid of possible newline characters
            temperature_str = result.stdout.strip()

            # Convert data to float
            temperature = int(temperature_str)

            # We get values in milli°C (thousandths of a Celsius), so we convert it to Celsius
            temperature = temperature / 1000

            return temperature  # In Celsius
        except Exception as error:
            print(f"ERROR: Failed to get temperature: {error}")
            return 0.0  # In Celsius
