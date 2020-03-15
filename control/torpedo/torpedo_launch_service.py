#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import odroid_wiringpi as wpi


TORPEDO_PIN = 0

def run():

    INPUT = 0
    OUTPUT = 1

    wpi.wiringPiSetup()
    wpi.pinMode(TORPEDO_PIN, OUTPUT)

    torpedo_service = rospy.Service('torpedo_launch', Trigger, torpedo_launch_cd)
    torpedo_service.spin()


def torpedo_launch_cd(trigger_request):
    """
    Should either 1) set a GPIO pin to high or 2) produce a certain pwm signal on a GPIO.
    Both should be stopped after a certain amout of time, that has to be determined by 
    testing. 
    """

    HIGH = 1
    LOW = 0

    # example code
    wpi.digitalWrite(TORPEDO_PIN, HIGH)
    rospy.sleep(3) # 3 seconds
    wpi.digitalWrite(TORPEDO_PIN, LOW)



if __name__ == "__main__":
    run()
