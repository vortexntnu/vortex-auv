#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool
from time import sleep
from datetime import datetime
import Jetson.GPIO as GPIO

# Note: is currently hardware dependent

ON = -1.0
OFF = 1.0
ACTIVE = 1
INACTIVE = 0

class GripperInterface:
    def __init__(self):

        self.gripper_state = inactive

        self.cooldown_period = 0.4  # Seconds
        self.last_press = datetime.now()

        # GPIO setup
        self.gripper_gpio_pin = 7
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.gripper_gpio_pin, GPIO.OUT)
        GPIO.output(self.gripper_gpio_pin, GPIO.LOW)

    def open_gripper(self):

        time_delta = datetime.now() - self.last_press
        if time_delta.total_seconds() > self.cooldown_period and self.gripper_state != active:
            GPIO.output(self.gripper_gpio_pin, GPIO.HIGH)
            rospy.loginfo("Gripper activated!")

            self.last_press = datetime.now()
            self.gripper_state = active

    def close_gripper(self):


        time_delta = datetime.now() - self.last_press
        if time_delta.total_seconds() > self.cooldown_period and self.gripper_state == active:
            GPIO.output(self.gripper_gpio_pin, GPIO.LOW)
            rospy.loginfo("Gripper deactivated!")

            self.last_press = datetime.now()
            self.gripper_state = inactive

    def shutdown(self):
        GPIO.output(self.gripper_gpio_pin, GPIO.LOW)
