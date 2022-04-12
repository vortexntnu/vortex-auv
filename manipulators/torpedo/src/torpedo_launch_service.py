#!/usr/bin/env python
# coding: UTF-8

import rospy
import Jetson.GPIO as GPIO
from vortex_msgs.srv import LaunchTorpedo


# How long to wait after a pin has been set to high
FIRING_DURATION = 3 # in seconds

# the two torpedos pin assignments. These have to be specified in a config file


class TorpedoLaunch():

    def __init__(self):
######## Definig the node ########        
        rospy.init_node('torpedo_node')

######## Defining the Service ########
        self.torpedo_service = rospy.Service('manipulator/torpedo_launch', LaunchTorpedo, execute)
        self.cooldown_period = 0.4 # Seconds
        self.last_press = datetime.now()
        self.torpedo_state = 'Back'

######## GPIO Setup ########
        torpedo_gpio_pin = rospy.get_param('torpedo_gpio')
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.torpedo_gpio_pin, GPIO.OUT)
        GPIO.output(self.torpedo_gpio_pin, GPIO.LOW)

    def execute(self, req):
        if req.fire == True and self.torpedo_state = 'Back':
            GPIO.output(self.torpedo_gpio_pin, GPIO.HIGH)
            self.last_press = datetime.now()
            self.torpedo_state = 'Front'

        elif req.fire == False and self.torpedo_state = 'Front':
            GPIO.output(self.torpedo_gpio_pin, GPIO.LOW)
            self.last_press = datetime.now()
            self.torpedo_state = 'Back'
        

if __name__ == "__main__":
    node = TorpedoLaunch()
    while not rospy.is_shutdown():
        rospy.spin()