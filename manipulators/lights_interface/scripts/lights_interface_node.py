#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from vortex_msgs.msg import Pwm
from datetime import datetime

# TODO:  have multiple light intensity lvls using pwm

pressed = 1
active = 1
inactive = 0

PWM_LOW = 1100
PWM_HIGH = 1900

class LightsInterfaceNode:
    def __init__(self):
        rospy.init_node('lights_interface')

        self.js_sub = rospy.Subscriber('/joystick/joy', Joy, self.callback, queue_size=1)
        self.pwm_pub = rospy.Publisher('/pwm', Pwm, queue_size=1)

        self.pwm_pin = 8
        self.light_state = inactive

        self.cooldown_period = 0.4 # Seconds
        self.last_press = datetime.now()

    def callback(self, joy_msg):
        
        button = joy_msg.buttons[7]
        
        if button == pressed: # only handle non-zero messages, since the joy topic is spammed
            time_delta = datetime.now() - self.last_press
            if time_delta.total_seconds() > self.cooldown_period:

                if self.light_state != active:
                    rospy.loginfo("Lights on!")
                    self.publish_pwm_msg(PWM_HIGH)
                    self.last_press = datetime.now()
                    self.light_state = active

                elif self.light_state == active:
                    rospy.loginfo("Lights off!")
                    self.publish_pwm_msg(PWM_LOW)
                    self.last_press = datetime.now()
                    self.light_state = inactive

    def publish_pwm_msg(self, us):
            msg = Pwm()
            msg.pins = [self.pwm_pin]
            msg.positive_width_us = [us]

            self.pwm_pub.publish(msg)

    # If PWM set is not sticky, have a spin function that writes

if __name__ == '__main__':
    node = LightsInterfaceNode()
    
    while not rospy.is_shutdown():
        rospy.spin()
