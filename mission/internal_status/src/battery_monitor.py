#!/usr/bin/env python

# python imports
import subprocess
import os
import serial

# ros imports
import rospy
from std_msgs.msg import Float32


class BatteryMonitor:
    def __init__(self):

        rospy.init_node("battery_monitor")

        # Parameters
        self.path_to_xavier_measurement = rospy.get_param("/battery/logging/path")
        self.path_to_powersense = rospy.get_param("/battery/logging/powersense_dev")
        self.critical_level = rospy.get_param("/battery/thresholds/critical")
        self.warning_level = rospy.get_param("/battery/thresholds/warning")
        system_interval = rospy.get_param("/battery/system/interval", 0.1)
        xavier_interval = rospy.get_param("/battery/xavier/interval", 10)
        logging_interval = rospy.get_param("/battery/logging/interval", 10)

        # Local variables
        self.xavier_voltage = 0.0
        self.system_voltage = 0.0

        # Power Sense device for system voltage
        self.powersense_device = serial.Serial(self.path_to_powersense, 115200)
        self.powersense_device.reset_input_buffer()

        # set up callbacks
        rospy.Timer(rospy.Duration(secs=system_interval), self.system_cb)
        rospy.Timer(rospy.Duration(secs=xavier_interval), self.xavier_cb)
        rospy.Timer(rospy.Duration(secs=logging_interval), self.log_cb)

        # Publishers
        self.xavier_battery_level_pub = rospy.Publisher(
            "/auv/battery_level/xavier", Float32, queue_size=1
        )
        self.system_battery_level_pub = rospy.Publisher(
            "/auv/battery_level/system", Float32, queue_size=1
        )

        rospy.loginfo("BatteryMonitor initialized")

    def xavier_cb(self):
        """Record output from voltage meter command, decode from bytes object to string, convert from string to integer"""
        xavier_mV = int(
            subprocess.check_output(["cat", self.path_to_voltage_meter]).decode("utf-8")
        )
        self.xavier_voltage = xavier_mV / 1000.0

        self.xavier_battery_level_pub.publish(self.xavier_voltage)

    def system_cb(self):
        """Read voltage of system from powersense device."""
        system_voltage_str = self.powersense_device.readline()
        self.system_voltage = float(system_voltage_str[:-2])  # strip /r/n
        self.powersense_device.reset_input_buffer()
        # readline only reads the top line, so make sure
        # the buffer is not filled with old voltage readings
        # by resetting the input buffer

        self.system_battery_level_pub.publish(self.system_voltage)

    def log_cb(self):
        self.log_voltage(xavier_voltage, "xavier")
        self.log_voltage(system_voltage, "system")

    def log_voltage(self, voltage, title):

        if voltage == 0:
            rospy.loginfo("Voltage is zero. Killswitch is probably off.")

        # Critical voltage level
        elif voltage <= self.critical_level:
            rospy.logerr("Critical %s voltage: %.3fV" % (title, voltage))

        # Warning voltage level
        elif voltage <= self.warning_level:
            rospy.logwarn("%s voltage: %.3fV" % (title, voltage))

        else:
            rospy.loginfo("%s voltage: %.3fV" % (title, voltage))

    def shutdown(self):
        self.powersense_device.close()


if __name__ == "__main__":
    bm = BatteryMonitor()
    try:
        rospy.spin()
    finally:
        bm.shutdown()
