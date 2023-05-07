#!/usr/bin/python3

# python imports
import subprocess
import re

# ros imports
import rospy
from std_msgs.msg import Float32

import board
import adafruit_mprls

i2c_adress_MPRLS = 0x18  # Reads pressure from MPRLS Adafruit sensor


class PressureMonitor:

    def __init__(self):
        rospy.init_node("pressure_monitor")

        # Publisher
        self.pressure_monitor_pub = rospy.Publisher("/auv/internal_pressure",
                                                    Float32,
                                                    queue_size=1)

        self.channel_pressure = adafruit_mprls.MPRLS(board.I2C(),
                                                     addr=i2c_adress_MPRLS,
                                                     reset_pin=None,
                                                     eoc_pin=None,
                                                     psi_min=0,
                                                     psi_max=25)  # Pressure

    def measure_pressure(self):
        return self.channel_pressure.pressure

    def spin(self):
        # Main loop
        while not rospy.is_shutdown():
            self.pressure_monitor_pub.publish(self.measure_pressure())

    def shutdown(self):
        pass


if __name__ == "__main__":
    pm = PressureMonitor()
    try:
        rospy.spin()
    finally:
        pm.shutdown()
