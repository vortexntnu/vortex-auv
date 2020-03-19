#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import odroid_wiringpi as wpi


"""

Service that lauches torpedos individually. 

"""


# enums for clearifying code
INPUT = 0
OUTPUT = 1
HIGH = 1
LOW = 0

# the two torpedos pin assignments. These have to be specified in a config file
torpedo_left = rospy.get_param('torpedo_pin_left')
torpedo_right = rospy.get_param('torpedo_pin_right')


def torpedo_launch_cd(trigger_request):
    """
    Should either 1) set a GPIO pin to high or 2) produce a certain pwm signal on a GPIO.
    Both should be stopped after a certain amout of time, that has to be determined by 
    testing. 
    """

    # example code
    wpi.digitalWrite(torpedo_left, HIGH)
    rospy.sleep(3) # 3 seconds
    wpi.digitalWrite(torpedo_left, LOW)





wpi.wiringPiSetup()
wpi.pinMode(torpedo_left, OUTPUT)
wpi.pinMode(torpedo_right, OUTPUT)

torpedo_service = rospy.Service('torpedo_launch', Trigger, torpedo_launch_cd)
torpedo_service.spin()
