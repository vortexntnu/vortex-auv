#!/usr/bin/env python
# coding: UTF-8

"""
"""

import rospy
from std_msgs.msg import String

def monitor_battery():
    voltage_sub = rospy.Subscriber('/tx_voltage_0', String, voltage_cb, queue_size=1)
    rospy.spin()


def voltage_cb(voltage_reading):
    pass


if __name__ == "__main__":
    try:
        monitor_battery()
    except:
        pass