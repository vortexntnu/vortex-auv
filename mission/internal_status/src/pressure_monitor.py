#!/usr/bin/python3
import os
import time

import rospy
from std_msgs.msg import Float32

import board
import adafruit_mprls


class PressureMonitor:
    def __init__(self):
        """
        Before we can set upp anything we need to kill a task and execute it again
        """

        # Sensor setup
        i2c_adress_MPRLS = 0x18  # Reads pressure from MPRLS Adafruit sensor
        self.channel_pressure = adafruit_mprls.MPRLS(
            board.I2C(),
            addr=i2c_adress_MPRLS,
            reset_pin=None,
            eoc_pin=None,
            psi_min=0,
            psi_max=25,
        )  # Pressure
        time.sleep(1)

        # ROS setup
        rospy.init_node("pressure_monitor")

        # Variables for publishing data
        self.pressure = 0

        # Variables for message handling
        self.buffer = 0
        self.buffer_max = 100

        # Create ROS publishers
        self.pressure_monitor_pub = rospy.Publisher(
            "/auv/internal_pressure", Float32, queue_size=1
        )

        # Getting params in the ROS-config file (beluga.yaml)
        self.critical_level = rospy.get_param(
            "/pressure/thresholds/critical", default=1013.25
        )

        # Set up time interval for publishing pressure data so that we dont spam ROS
        system_interval = rospy.get_param("/pressure/system/interval", default=1)
        self.system_timer = rospy.Timer(
            rospy.Duration(secs=system_interval),
            self.system_call_back,  # will update and publish measurements to ROS
        )

        # Set up time interval for loging errors
        loger_interval = rospy.get_param("/pressure/loger/interval", default=5)
        self.loger_timer = rospy.Timer(
            rospy.Duration(secs=loger_interval),
            self.loger_call_back,  # will monitor for any pressure errors and display it on screen
        )

    def measure_pressure(self):
        try:
            self.pressure = self.channel_pressure.pressure
        except:
            rospy.logerr(f"Couldn't get pressure data, trying again...")

    def system_call_back(self, event):
        self.measure_pressure()

        self.pressure_monitor_pub.publish(self.pressure)

    def loger_call_back(self, event):
        if self.pressure > self.critical_level:
            rospy.logerr(
                f"The internal pressure to HIGH: {self.pressure} hPa! Drone might be leaking!"
            )

    def shutdown(self):
        pass


if __name__ == "__main__":
    pm = PressureMonitor()
    try:
        rospy.spin()
    finally:
        pm.shutdown()
