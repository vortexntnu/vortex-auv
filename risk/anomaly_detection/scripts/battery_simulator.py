#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
from diagnostic_msgs.msg import *
from std_msgs.msg import Float32
from vortex_msgs.srv import *
import dynamic_reconfigure.server
from anomaly_detection.cfg import BatterySimulatorConfig
import thread

class BatterySimulator():

    def __init__(self):
        pass


if __name__ == '__main__':

    BatterySimulator()