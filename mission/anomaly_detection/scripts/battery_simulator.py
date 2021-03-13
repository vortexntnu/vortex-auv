#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
from diagnostic_msgs.msg import *
from std_msgs.msg import Float32
from vortex_msgs.srv import *
from dynamic_reconfigure.server import Server
from anomaly_detection.cfg import BatterySimulatorConfig
import thread

class BatterySimulator():

    def __init__(self):
        
        rospy.init_node("battery_simulator")

        # get parameters
        self.rate = rospy.get_param('~rate', 1)
        self.battery_runtime = rospy.get_param("~battery_runtime", 400) # seconds
        self.initial_battery_level = rospy.get_param('~initial_battery_level', 100) # percentage
        self.error_battery_level = rospy.get_param('~error_battery_level', 20) # percentage
        self.warn_battery_level = rospy.get_param('~warn_battery_level', 50) # percentage

        # set parameters
        ros_rate = rospy.Rate(self.rate)
        self.current_battery_level = self.initial_battery_level
        self.new_battery_level = self.initial_battery_level

        # the step size to decrease battery level
        self.battery_step = float(self.initial_battery_level) / self.rate / self.battery_runtime    

        #  mutually exclusive flag to enforce mutual exclusion concurrency control policy
        self.mutex = thread.allocate_lock()

        # topics
        self.battery_level_pub = rospy.Publisher("/auv/battery_level", Float32, queue_size=1)
        self.diag_pub = rospy.Publisher("/auv/diagnostics", DiagnosticArray, queue_size=1)

        # services
        # manually set battery level
        rospy.Service('~set_battery_level', SetBatteryLevel, self.SetBatteryLevelHandler)

        # create a dynamic reconfigure server and set a callback function
        self.battery_dyn_reconfigure = Server(BatterySimulatorConfig, self.dynamic_reconfigure_callback)

        while not rospy.is_shutdown():

            # initialize the diagnostics status
            status = DiagnosticStatus()
            status.name = "battery_level"

            # Set the diagnostics level based on the current battery level
            if self.current_battery_level < self.error_battery_level:
                status.message = "Low Battery"
                status.level = DiagnosticStatus.ERROR
            elif self.current_battery_level < self.warn_battery_level:
                status.message = "Medium Battery"
                status.level = DiagnosticStatus.WARN
            else:
                status.message = "Battery OK"
                status.level = DiagnosticStatus.OK

            # add the current raw battery level to the diagnostics message
            status.values.append(KeyValue("Battery Level", str(self.current_battery_level)))

            # build the diagnostics array message
            msg = DiagnosticArray()
            msg.header.stamp = rospy.Time.now()
            msg.status.append(status)
            
            # publish
            self.diag_pub.publish(msg)
            self.battery_level_pub.publish(self.current_battery_level)

            # adjust current battery level
            self.current_battery_level = max(0, self.current_battery_level - self.battery_step)

            ros_rate.sleep()

    def dynamic_reconfigure_callback(self, config, level):

        if self.battery_runtime != config['battery_runtime']:
            self.battery_runtime = config['battery_runtime']
            self.battery_step = 100.0 / self.rate / self.battery_runtime

        if self.new_battery_level != config['new_battery_level']:
            self.new_battery_level = config['new_battery_level']
            self.mutex.acquire() # set lock
            self.current_battery_level = self.new_battery_level
            self.mutex.release() # release lock

        return config

    def SetBatteryLevelHandler(self, req):
        self.mutex.acquire() # set lock
        self.current_battery_level = req.value
        self.mutex.release() # release lock
        return SetBatteryLevelResponse()



if __name__ == '__main__':

    BatterySimulator()