#!/usr/bin/env python
# coding: UTF-8

import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Int32

PIN = 15

class TorpedoLaunch:
    def __init__(self):
        rospy.init_node("torpedo_node")

        rospy.Subscriber("/torpedo", Int32, self.execute, queue_size=1)

        self.mode = GPIO.HIGH
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN, GPIO.OUT, initial=self.mode)

    def execute(self, msg):
        if self.mode == GPIO.HIGH:
            print("going to low")
            GPIO.output(PIN, GPIO.LOW)
            self.mode = GPIO.LOW

        elif self.mode == GPIO.LOW:
            print("going to high")
            GPIO.output(PIN, GPIO.HIGH)
            self.mode = GPIO.HIGH



if __name__ == "__main__":
    node = TorpedoLaunch()
    rospy.spin()
