#!/usr/bin/python3

import os
import time
import traceback

import rospy
import smbus
from std_msgs.msg import Float32


class BatteryMonitor:
    def __init__(self):

        rospy.init_node("battery_monitor")

        # Parameters
        self.path_to_powersense = rospy.get_param(
            "/battery/system/path", default="/dev/i2c-1"
        )

        self.i2c_address_powersense_voltage = rospy.get_param(
            "/i2c/psm/address_voltage", default=0x6A
        )

        self.i2c_bus_number = rospy.get_param("/i2c/psm/bus_number", default=1)
        rospy.loginfo(
            "PSM Voltage I2C address: '0x{:02x}'".format(
                self.i2c_address_powersense_voltage
            )
        )
        rospy.loginfo("PSM I2C bus number: '%d'", self.i2c_bus_number)

        self.critical_level = rospy.get_param(
            "/battery/thresholds/critical", default=13.5
        )
        self.warning_level = rospy.get_param(
            "/battery/thresholds/warning", default=14.5
        )

        # Polling intervals in seconds delay
        system_interval = rospy.get_param("/battery/system/interval", 1)
        logging_interval = rospy.get_param("/battery/logging/interval", 5)

        # Local variables
        self.system_voltage = 0.0
        self.system_recieved = False

        self.is_PSM_fuckd_voltage = False

        # Get I2C bus for power sense module
        self.bus = smbus.SMBus(self.i2c_bus_number)
        time.sleep(1)

        # Send configure command to the module to enable continuous conversion in 12-bit mode
        try:
            self.bus.write_byte(self.i2c_address_powersense_voltage, 0x10)
            time.sleep(0.5)
        except IOError:
            print(traceback.format_exc())
            self.is_PSM_fuckd_voltage = True

        if not self.is_PSM_fuckd_voltage:
            self.system_battery_level_pub = rospy.Publisher(
                "/auv/battery_level/system", Float32, queue_size=1
            )
        # set up callbacks
        self.log_timer = rospy.Timer(rospy.Duration(secs=logging_interval), self.log_cb)

        if not self.is_PSM_fuckd_voltage:
            self.system_timer = rospy.Timer(
                rospy.Duration(secs=system_interval), self.system_cb
            )
            rospy.loginfo("BatteryMonitor initialized")
        else:
            rospy.logwarn("System Battery Monitoring from PSM is not available.")

    def system_cb(self, event):
        """Read voltage of system from powersense device."""

        # MCP3425 address, 0x68(104)
        # Read data back from 0x00(00), 2 bytes, MSB first
        # raw_adc MSB, raw_adc LSB
        if self.is_PSM_fuckd_voltage:
            return

        voltage_msg = self.bus.read_i2c_block_data(
            self.i2c_address_powersense_voltage, 0x00, 2
        )

        # Convert the data to 12-bits
        raw_adc_voltage = (voltage_msg[0] & 0x0F) * 256 + voltage_msg[1]
        if raw_adc_voltage > 2047:
            raw_adc_voltage -= 4095

        # PSM specific conversion ratio
        self.system_voltage = raw_adc_voltage * 0.011
        self.system_battery_level_pub.publish(self.system_voltage)

        if self.system_voltage < self.critical_level:
            rospy.logerr(
                f"Critical voltage: {self.system_voltage}V! Shutting down all active nodes!"
            )
            os.system("rosnode kill -a")

        if not self.system_recieved:
            self.system_recieved = True

    def log_cb(self, event):

        if self.system_recieved:
            self.log_voltage(self.system_voltage, "system")
        else:
            rospy.loginfo("No voltage recieved from system yet.")

    def log_voltage(self, voltage, title):

        if voltage == 0:
            rospy.loginfo("Voltage is zero. Killswitch is probably off.")

        elif voltage <= self.warning_level:
            rospy.logwarn("%s voltage: %.3fV" % (title, voltage))

        else:
            rospy.loginfo("%s voltage: %.3fV" % (title, voltage))

    def shutdown(self):
        if not self.is_PSM_fuckd_voltage:
            self.system_timer.shutdown()
            self.log_timer.shutdown()
            self.bus.close()


if __name__ == "__main__":
    bm = BatteryMonitor()
    try:
        rospy.spin()
    finally:
        bm.shutdown()
