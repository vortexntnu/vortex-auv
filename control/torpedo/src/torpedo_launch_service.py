#!/usr/bin/env python
# coding: UTF-8

import rospy
import RPi.GPIO as GPIO
from vortex_msgs.srv import LaunchTorpedo


# How long to wait after a pin has been set to high
FIRING_DURATION = 3 # in seconds

# the two torpedos pin assignments. These have to be specified in a config file
torpedo_left_pin = rospy.get_param('tordedos/left_pin')
torpedo_right_pin = rospy.get_param('tordedos/right_pin')


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(torpedo_left_pin, GPIO.OUT)
    GPIO.setup(torpedo_right_pin, GPIO.OUT)


def turn_off():
    """
    Turns off pins gracefully
    """
    GPIO.output(torpedo_left_pin, GPIO.LOW)
    GPIO.output(torpedo_right_pin, GPIO.LOW)
    GPIO.cleanup()


def torpedo_launch_cd(request):

    if request.torpedo_selection == 'LEFT':
        GPIO.output(torpedo_left_pin, GPIO.HIGH)
        rospy.sleep(FIRING_DURATION)
        GPIO.output(torpedo_left_pin, GPIO.LOW)

    elif request.torpedo_selection == 'RIGHT':
        GPIO.output(torpedo_right_pin, GPIO.HIGH)
        rospy.sleep(FIRING_DURATION)
        GPIO.output(torpedo_right_pin, GPIO.LOW)

    else:
        raise ValueError('torpedo launch service recieved a torpedo selection other than LEFT or RIGHT')


if __name__ == "__main__":
    setup()
    try:
        torpedo_service = rospy.Service('torpedo_launch', LaunchTorpedo, torpedo_launch_cd)
        torpedo_service.spin()
    finally:
        turn_off()
