#!/usr/bin/env python
# coding: UTF-8

"""
Node that reads battery voltage from the jetson tx2 and sends out a warning if
battery voltage is too low. 
This node depends on "tx_battery_voltage_publisher" running on the tx.
"""

import rospy
from std_msgs.msg import UInt32

class BatteryMonitor():

    def __init__(self):
        self.voltag_limit = rospy.get_param('voltage_limit', default=13500)
        self.voltage_sub = rospy.Subscriber('/tx_voltage_0', UInt32, self.voltage_cb, queue_size=1)
        rospy.logdebug("Battery monitor running..")

    def voltage_cb(self, voltage_reading):
        if voltage_reading < self.voltag_limit:
            rospy.logwarn("Battery voltage is too low. Charge immediately. Current voltage [mV]: " + voltage_reading)


if __name__ == "__main__":
    try:
        battery_monitor = BatteryMonitor()
        rospy.spin()
    except:
        rospy.logdebug("Battery monitor is shut down")