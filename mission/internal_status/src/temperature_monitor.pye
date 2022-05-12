#!/usr/bin/python3

#python imports
import subprocess
import re

#ros imports
import rospy
from std_msgs.msg import Int32

class TemperatureMonitor():
    def __init__(self):
        
        rospy.init_node("temperature_monitor")

        # Settings
        self.interval = rospy.get_param("/temperature/logging/interval")          # How often the battery level is checked and published
        self.temperature_template = r"\d*\.{0,1}\d*" # In python file because yaml doesn't like escape characters
        
        # Publisher
        self.cpu_temperature_pub = rospy.Publisher("/auv/temperature/cpu", Int32, queue_size=1)
        self.gpu_temperature_pub = rospy.Publisher("/auv/temperature/gpu", Int32, queue_size=1)
        
        # Start the tegrastats subprocess
        self.process = subprocess.Popen("tegrastats", shell=False, stdout=subprocess.PIPE)

    def measure_temp(self):

        # Record output from temperature meter command, decode from bytes object to string, convert from string to integer

        stats = self.process.stdout.readlines()[-1].decode("utf-8")

        if stats != "":

            cpu_search = re.search("CPU@("+self.temperature_template+")C", stats)
            gpu_search = re.search("GPU@("+self.temperature_template+")C", stats)

            self.cpu_temperature = int(float(cpu_search.group(1)))
            self.gpu_temperature = int(float(gpu_search.group(1)))


    def spin(self):
        # Main loop
        while not rospy.is_shutdown():

            self.measure_temp()
            self.cpu_temperature_pub.publish(self.cpu_temperature)
            self.gpu_temperature_pub.publish(self.gpu_temperature)

            rospy.sleep(self.interval)

if __name__ == "__main__":

    tm = TemperatureMonitor()
    tm.spin()
