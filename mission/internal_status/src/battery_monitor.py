#!/usr/bin/env python

# python imports
import subprocess
import os
import ctypes
import pylibi2c

# ros imports
import rospy
from std_msgs.msg import Float32


class BatteryMonitor:
    def __init__(self):

        rospy.init_node("battery_monitor")

        # Parameters
        self.path_to_xavier_measurement = rospy.get_param("/battery/xavier/path", default="/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_voltage0_input")
        self.path_to_powersense = rospy.get_param("/battery/system/path", default="/dev/i2c-8")
        
        self.i2c_address_powersense_voltage = rospy.get_param("/i2c/psm/address_voltage")
        self.i2c_address_powersense_current = rospy.get_param("/i2c/psm/address_current")
        self.i2c_bus_powersense = rospy.get_param("/i2c/psm/bus")
        
        self.critical_level = rospy.get_param("/battery/thresholds/critical", default=13.5)
        self.warning_level = rospy.get_param("/battery/thresholds/warning", default=14.5)
        
        system_interval = rospy.get_param("/battery/system/interval", 0.05)
        xavier_interval = rospy.get_param("/battery/xavier/interval", 10)
        logging_interval = rospy.get_param("/battery/logging/interval", 10)

        # Local variables
        self.xavier_voltage = 0.0
        self.system_voltage = 0.0
        self.system_current = 0.0
        
        self.xavier_recieved = False
        self.system_recieved_voltage = False
        self.system_recieved_current = False

        # Power Sense device for system voltage
        self.powersense_device_voltage = pylibi2c.I2CDevice(self.path_to_powersense, self.i2c_address_powersense_voltage)
        self.powersense_device_current = pylibi2c.I2CDevice(self.path_to_powersense, self.i2c_address_powersense_current)

        self.voltage_buffer = bytes(bytearray(256))
        self.current_buffer = bytes(bytearray(256))

        # set up callbacks
        self.system_timer = rospy.Timer(rospy.Duration(secs=system_interval), self.system_cb)
        self.xavier_timer = rospy.Timer(rospy.Duration(secs=xavier_interval), self.xavier_cb)
        self.log_timer = rospy.Timer(rospy.Duration(secs=logging_interval), self.log_cb)

        # Publishers
        self.xavier_battery_level_pub = rospy.Publisher(
            "/auv/battery_level/xavier", Float32, queue_size=1
        )
        self.system_battery_level_pub = rospy.Publisher(
            "/auv/battery_level/system", Float32, queue_size=1
        )

        rospy.loginfo("BatteryMonitor initialized")

    def xavier_cb(self, event):
        """Record output from voltage meter command, decode from bytes object to string, convert from string to integer"""
        xavier_mV = int(
            subprocess.check_output(["cat", self.path_to_xavier_measurement]).decode("utf-8")
        )
        self.xavier_voltage = xavier_mV / 1000.0
        
        self.xavier_battery_level_pub.publish(self.xavier_voltage)
        self.xavier_recieved = True

    def system_cb(self, event):
        """Read voltage of system from powersense device."""
        self.system_voltage = self.powersense_device_voltage.ioctl_read(0x0, 256)
        self.system_current = self.powersense_device_current.ioctl_read(0x0, 256)

        # readline only reads the top line, so make sure
        # the buffer is not filled with old voltage readings
        # by resetting the input buffer

        self.system_battery_level_pub.publish(self.system_voltage)
        self.system_recieved = True

    def log_cb(self, event):
        
        if self.xavier_recieved:
            self.log_voltage(self.xavier_voltage, "xavier")
        else:
            rospy.loginfo("No voltage recieved from Xavier yet.")
            
        if self.system_recieved:
            self.log_voltage(self.system_voltage, "system")
        else:
            rospy.loginfo("No voltage recieved from system yet.")
            

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
        self.system_timer.shutdown()
        self.xavier_timer.shutdown()
        self.log_timer.shutdown()
        self.powersense_device.close()


if __name__ == "__main__":
    bm = BatteryMonitor()
    try:
        rospy.spin()
    finally:
        bm.shutdown()
