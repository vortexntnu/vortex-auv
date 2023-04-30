#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from time import sleep
from datetime import datetime
import RPi.GPIO as GPIO

# TODO: Need different nodes/topics/modes for ROV and AUV operations
# Note: is currently hardware dependent; replace GPIO calls with ros publish at some point,
# and create a driver.

on = -1.0
off = 1.0
active = 1
inactive = 0

GRIPPER_PIN1 = 6
GRIPPER_PIN2 = 5


class GripperInterfaceNode:

    def __init__(self):
        rospy.init_node("gripper_interface")
        self.joystick_sub = rospy.Subscriber("/mission/joystick_data",
                                             Joy,
                                             self.callback,
                                             queue_size=1)

        self.gripper_state = inactive

        self.cooldown_period = 0.4  # Seconds
        self.last_press = datetime.now()

        # GPIO setup
        self.gripper_gpio_pin1 = GRIPPER_PIN1
        self.gripper_gpio_pin2 = GRIPPER_PIN2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gripper_gpio_pin1, GPIO.OUT)
        GPIO.setup(self.gripper_gpio_pin2, GPIO.OUT)
        GPIO.output(self.gripper_gpio_pin1, GPIO.HIGH)
        GPIO.output(self.gripper_gpio_pin2, GPIO.HIGH)

    def callback(self, joy_msg):
        Dpad = joy_msg.axes[7]
        if Dpad != 0:  # only handle non-zero messages, since the joy topic is spammed
            time_delta = datetime.now() - self.last_press
            if time_delta.total_seconds() > self.cooldown_period:
                if Dpad == on and self.gripper_state != active:
                    GPIO.output(self.gripper_gpio_pin1, GPIO.LOW)
                    GPIO.output(self.gripper_gpio_pin2, GPIO.LOW)
                    rospy.loginfo("Gripper activated!")

                    self.last_press = datetime.now()
                    self.gripper_state = active

                elif Dpad == off and self.gripper_state == active:
                    GPIO.output(self.gripper_gpio_pin1, GPIO.HIGH)
                    GPIO.output(self.gripper_gpio_pin2, GPIO.HIGH)
                    rospy.loginfo("Gripper deactivated!")

                    self.last_press = datetime.now()
                    self.gripper_state = inactive

    def shutdown(self):
        GPIO.output(self.gripper_gpio_pin1, GPIO.HIGH)
        GPIO.output(self.gripper_gpio_pin2, GPIO.HIGH)


if __name__ == "__main__":
    node = GripperInterfaceNode()

    while not rospy.is_shutdown():
        rospy.spin()

    node.shutdown()
