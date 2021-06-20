#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from vortex_msgs.msg import Pwm

# TODO: get data from controller, implement btn bounceback, have multiple light intensity lvls using pwm
# TODO: sub/pub node --> sub to joystick, pub to PCA using pwm
# TODO: pub using pwm msg type - pin 8
# TODO: subscribe to lights pwm topic from PCA interface node

class LightsInterface:
    def __init__(self):
        self.btn_status = 0
        self.btn_status = 0

        self.lights_pwm = 0

        self.js_sub = rospy.Subscriber('/mission/joystick_data', Joy, self.callback, queue_size=1)
        self.pwm_pub = rospy.Publisher('/manipulators/lights', Pwm, queue_size=1)