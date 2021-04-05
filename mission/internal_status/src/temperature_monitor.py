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

        # Settings
        self.path_to_cpu_temperature_meter = rospy.get_param("/temperature/logging/paths/cpu")
        self.path_to_gpu_temperature_meter = rospy.get_param("/temperature/logging/paths/gpu")
        self.interval = rospy.get_param("/temperature/logging/interval")          # How often the battery level is checked and published
        
        # Publisher
        self.cpu_temperature_pub = rospy.Publisher("/auv/temperature/cpu", Int32, queue_size=1)
        self.gpu_temperature_pub = rospy.Publisher("/auv/temperature/gpu", Int32, queue_size=1)
        
    def spin(self):
        # Main loop
        while not rospy.is_shutdown():

            self.measure_temp()
            self.cpu_temperature_pub.publish(self.cpu_temperature)
            self.gpu_temperature_pub.publish(self.gpu_temperature)

            rospy.sleep(self.interval)

    def measure_temp(self):

        # Record output from temperature meter command, decode from bytes object to string, convert from string to integer
        self.cpu_temperature = int(subprocess.check_output(["cat", self.path_to_cpu_temperature_meter]).decode("utf-8"))
        self.gpu_temperature = int(subprocess.check_output(["cat", self.path_to_gpu_temperature_meter]).decode("utf-8"))
        
        
if __name__ == "__main__":
    tm = TemperatureMonitor()
    tm.spin()
