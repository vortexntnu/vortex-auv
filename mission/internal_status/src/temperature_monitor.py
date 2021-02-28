#!/usr/bin/env python

#python imports
import subprocess
import os

#ros imports
import rospy
from std_msgs.msg import Int32

class TemperatureMonitor():
    def __init__(self):
        
        rospy.init_node("temperature_monitor")
        
        # Publisher
        self.temperature_pub = rospy.Publisher("/auv/temperature", Int32, queue_size=1)
        
        # Main loop
        while not rospy.is_shutdown():

            self.measure_temp()
            self.temperature_pub.publish(self.temperature)

    def measure_temp(self):
        # Placehodler until test
        self.temperature = 0