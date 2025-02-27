import board
import time

import smbus
from MCP342x import MCP342x


class GripperFeedback:

    def __init__(self):
        # Pressure Sensor Setup
        i2c_adress_MPRLS = 0x21  # Gripper mcu has address 0x21
        self.i2c_channel = None

        # init of I2C bus communication
        self.bus = None
        try:
            self.bus = smbus.SMBus(0)  # check if channel 0 is open
        except Exception as error:
            print(f"ERROR: Failed to connect to the I2C: {error}")
        time.sleep(1)

        try:
            self.i2c_channel = MCP342x(self.bus,
                                       self.i2c_adress,
                                       channel=0,
                                       resolution=18)
        except Exception as error:
            print(f"ERROR: Failed connecting to PSM: {error}")

        time.sleep(1)

    def get_feedback(self) -> list[float]:
        try:
            gripper_values = self.channel_pressure.convert_and_read()
            return gripper_values
        except Exception as error:
            print(f"ERROR: Failed to get gripper feedback: {error}")
            return [0.0, 0.0, 0.0]
