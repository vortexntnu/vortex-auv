#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from vortex_msgs.msg import Pwm

# TODO:  have multiple light intensity lvls using pwm
# TODO: subscribe to lights pwm topic from PCA interface node

on = 1
off = -1
active = 1
inactive = 0

PWM_LOW = 1100
PWM_HIGH = 1900

class LightsInterfaceNode:
    def __init__(self):
        rospy.init_node('/manipulators/lights_interface')

        self.js_sub = rospy.Subscriber('/mission/joystick_data', Joy, self.callback, queue_size=1)
        self.pwm_pub = rospy.Publisher('/manipulators/lights', Pwm, queue_size=1)

        self.pwm_pin = 8
        self.light_state = off

        self.cooldown_period = 2 # Seconds
        self.last_press = datetime.now()

    def callback(self, joy_msg):
        
        Dpad = joy_msg.axes[6]

        time_since_press = self.last_press - datetime.now()
        if time_since_press > self.cooldown_period:

            if Dpad == on and self.light_state != active:
                self.pwm_pub.publish(PWM_HIGH)
                self.last_press = datetime.now()
                self.light_state = active

            elif Dpad == off and self.light_state == active:
                self.pwm_pub.publish(PWM_LOW)
                self.last_press = datetime.now()
                self.light_state = inactive

    def publish_pwm_msg(self, us):
            msg = Pwm()
            msg.pins = self.pwm_pin
            msg.positive_width_us = us

            self.pwm_pub.publish(msg)

    # If PWM set is not sticky, have a spin function that writes

if __name__ == '__main__':
    node = LightsInterfaceNode()
    
    while not rospy.is_shutdown():
        rospy.spin()