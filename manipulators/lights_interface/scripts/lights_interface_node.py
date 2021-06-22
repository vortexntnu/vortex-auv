#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from vortex_msgs.msg import Pwm
from datetime import datetime

# TODO:  have multiple light intensity lvls using pwm
# TODO: subscribe to lights pwm topic from PCA interface node

on = -1.0
off = 1.0
active = 1
inactive = 0

PWM_LOW = 1100
PWM_HIGH = 1900

class LightsInterfaceNode:
    def __init__(self):
        rospy.init_node('lights_interface')

        self.js_sub = rospy.Subscriber('/mission/joystick_data', Joy, self.callback, queue_size=1)
        self.pwm_pub = rospy.Publisher('/manipulators/lights', Pwm, queue_size=1)

        self.pwm_pin = 8
        self.light_state = off

        self.cooldown_period = 2 # Seconds
        self.last_press = datetime.now()

    def callback(self, joy_msg):
        
        Dpad = joy_msg.axes[6]
        
        if Dpad != 0: # only handle non-zero messages, since the joy topic is spammed
            time_delta = datetime.now() - self.last_press
            if time_delta.total_seconds() > self.cooldown_period:

                if Dpad == on and self.light_state != active:
                    rospy.loginfo("Lights on!")
                    self.publish_pwm_msg(PWM_HIGH)
                    self.last_press = datetime.now()
                    self.light_state = active

                elif Dpad == off and self.light_state == active:
                    rospy.loginfo("Lights off!")
                    self.publish_pwm_msg(PWM_LOW)
                    self.last_press = datetime.now()
                    self.light_state = inactive

    def publish_pwm_msg(self, us):
            msg = Pwm()
            msg.pins = 8
            msg.positive_width_us = us

            self.pwm_pub.publish(msg)

    # If PWM set is not sticky, have a spin function that writes

if __name__ == '__main__':
    node = LightsInterfaceNode()
    
    while not rospy.is_shutdown():
        rospy.spin()