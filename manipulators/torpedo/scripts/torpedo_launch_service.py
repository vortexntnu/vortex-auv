#!/usr/bin/python3
# coding: UTF-8

import rospy
import Jetson.GPIO as GPIO
from vortex_msgs.srv import LaunchTorpedo, LaunchTorpedoResponse


# How long to wait after a pin has been set to high
FIRING_DURATION = 3  # in seconds

# the two torpedos pin assignments. These have to be specified in a config file


class TorpedoLaunch:
    def __init__(self):
        ######## Definig the node ########
        rospy.init_node("torpedo_node")

        ######## Defining the Service ########
        self.torpedo_service = rospy.Service(
            "manipulator/torpedo_launch", LaunchTorpedo, self.execute
        )

        ######## GPIO Setup ########
        self.mode = GPIO.LOW
        self.torpedo_gpio_pin = rospy.get_param("/torpedo/gpio_pin")
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.torpedo_gpio_pin, GPIO.OUT, initial=self.mode)

    def execute(self, req):
        if req.fire == True and self.mode == GPIO.HIGH:
            print("going to low")
            GPIO.output(self.torpedo_gpio_pin, GPIO.LOW)
            self.mode = GPIO.LOW

        elif req.fire == True and self.mode == GPIO.LOW:
            print("going to high")
            GPIO.output(self.torpedo_gpio_pin, GPIO.HIGH)
            self.mode = GPIO.HIGH

        res = LaunchTorpedoResponse()

        return res


if __name__ == "__main__":
    node = TorpedoLaunch()
    while not rospy.is_shutdown():
        rospy.spin()
