#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from time import sleep
from datetime import datetime
import Jetson.GPIO as GPIO

# TODO: Need different nodes/topics/modes for ROV and AUV operations
# Note: is currently hardware dependent; replace GPIO calls with ros publish at some point,
# and create a driver.

on = 1
off = -1
active = 1
inactive = 0

class GripperInterfaceNode():
    def __init__(self):
        rospy.init_node('/manipulators/gripper_interface')
        self.joystick_sub = rospy.Subscriber('/mission/joystick_data', Joy, self.callback, queue_size=1)

        self.gripper_state = inactive

        self.cooldown_period = 2 # Seconds
        self.last_press = datetime.now()

        # GPIO setup
        gripper_gpio_pin = 7
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(gripper_gpio_pin, GPIO.OUT)
        GPIO.output(gripper_gpio_pin, GPIO.LOW)

    def callback(self, joy_msg):
        
        Dpad = joy_msg.axes[7]
        
        time_since_press = self.last_press - datetime.now()
        if time_since_press > self.cooldown_period:

            if Dpad == on and self.gripper_state != active:
                GPIO.output(gripper_gpio_pin, GPIO.HIGH)

                self.last_press = datetime.now()
                self.gripper_state = active

            elif Dpad == off and self.gripper_state == active:
                GPIO.output(gripper_gpio_pin, GPIO.LOW)

                self.last_press = datetime.now()
                self.gripper_state = inactive


if __name__ == '__main__':
    node = GripperInterfaceNode()
    
    while not rospy.is_shutdown():
        rospy.spin()