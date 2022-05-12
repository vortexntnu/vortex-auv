#!/usr/bin/python3

import subprocess
import time
import traceback

import rospy
import smbus
from std_msgs.msg import Float32


class BatteryMonitor:
    def __init__(self):

        rospy.init_node("battery_monitor")

        # Parameters
        self.path_to_xavier_measurement = rospy.get_param(
            "/battery/xavier/path",
            default="/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_voltage0_input",
        )
        self.path_to_powersense = rospy.get_param(
            "/battery/system/path", default="/dev/i2c-8"
        )

        self.i2c_address_powersense_voltage = rospy.get_param(
            "/i2c/psm/address_voltage", default=0x6A
        )
        self.i2c_address_powersense_current = rospy.get_param(
            "/i2c/psm/address_current", default=0x69
        )
        self.i2c_bus_number = rospy.get_param("/i2c/psm/bus_number", default=8)
        rospy.loginfo(
            "PSM Voltage I2C address: '0x{:02x}'".format(
                self.i2c_address_powersense_voltage
            )
        )
        rospy.loginfo(
            "PSM Current I2C address: '0x{:02x}'".format(
                self.i2c_address_powersense_current
            )
        )
        rospy.loginfo("PSM I2C bus number: '%d'", self.i2c_bus_number)

        self.critical_level = rospy.get_param(
            "/battery/thresholds/critical", default=13.5
        )
        self.warning_level = rospy.get_param(
            "/battery/thresholds/warning", default=14.5
        )

        system_interval = rospy.get_param("/battery/system/interval", 0.05)
        xavier_interval = rospy.get_param("/battery/xavier/interval", 10)
        logging_interval = rospy.get_param("/battery/logging/interval", 10)

        # Local variables
        self.xavier_voltage = 0.0
        self.system_voltage = 0.0
        self.system_current = 0.0

        self.xavier_recieved = False
        self.system_recieved = False

        self.is_PSM_fuckd_voltage = False
        self.is_PSM_fuckd_current = False

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

        try:
            self.bus.write_byte(self.i2c_address_powersense_current, 0x10)
            time.sleep(0.5)
        except IOError:
            print(traceback.format_exc())
            self.is_PSM_fuckd_current = True

        # Publishers
        self.xavier_battery_level_pub = rospy.Publisher(
            "/auv/battery_level/xavier", Float32, queue_size=1
        )
        if not self.is_PSM_fuckd_voltage:
            self.system_battery_level_pub = rospy.Publisher(
                "/auv/battery_level/system", Float32, queue_size=1
            )
        if not self.is_PSM_fuckd_current:
            self.system_battery_current_draw_pub = rospy.Publisher(
                "/auv/battery_level/system_current_draw", Float32, queue_size=1
            )

        # set up callbacks

        self.xavier_timer = rospy.Timer(
            rospy.Duration(secs=xavier_interval), self.xavier_cb
        )
        self.log_timer = rospy.Timer(rospy.Duration(secs=logging_interval), self.log_cb)

        if not self.is_PSM_fuckd_voltage or not self.is_PSM_fuckd_current:
            self.system_timer = rospy.Timer(
                rospy.Duration(secs=system_interval), self.system_cb
            )
            rospy.loginfo("BatteryMonitor initialized")
        else:
            rospy.logwarn("System Battery Monitoring from PSM is not available.")

    def xavier_cb(self, event):
        """Record output from voltage meter command, decode from bytes object to string, convert from string to integer"""
        xavier_mV = int(
            subprocess.check_output(["cat", self.path_to_xavier_measurement]).decode(
                "utf-8"
            )
        )
        self.xavier_voltage = xavier_mV / 1000.0

        self.xavier_battery_level_pub.publish(self.xavier_voltage)
        if not self.xavier_recieved:
            self.xavier_recieved = True

    def system_cb(self, event):
        """Read voltage of system from powersense device."""

        # MCP3425 address, 0x68(104)
        # Read data back from 0x00(00), 2 bytes, MSB first
        # raw_adc MSB, raw_adc LSB
        if not self.is_PSM_fuckd_voltage:
            voltage_msg = self.bus.read_i2c_block_data(
                self.i2c_address_powersense_voltage, 0x00, 2
            )

            # Convert the data to 12-bits
            raw_adc_voltage = (voltage_msg[0] & 0x0F) * 256 + voltage_msg[1]
            if raw_adc_voltage > 2047:
                raw_adc_voltage -= 4095

        # MCP3425 address, 0x68(104)
        # Read data back from 0x00(00), 2 bytes, MSB first
        # raw_adc MSB, raw_adc LSB
        if not self.is_PSM_fuckd_current:
            current_msg = self.bus.read_i2c_block_data(
                self.i2c_address_powersense_current, 0x00, 2
            )
            # Convert the data to 12-bits
            raw_adc_current = (current_msg[0] & 0x0F) * 256 + current_msg[1]
            if raw_adc_current > 2047:
                raw_adc_current -= 4095

        # PSM specific conversion ratio
        if not self.is_PSM_fuckd_voltage:
            self.system_voltage = raw_adc_voltage * 0.011
        if not self.is_PSM_fuckd_current:
            self.system_current = raw_adc_current * 0.0504

        # publish
        if not self.is_PSM_fuckd_voltage:
            self.system_battery_level_pub.publish(self.system_voltage)
        if not self.is_PSM_fuckd_current:
            self.system_battery_current_draw_pub.publish(self.system_current)
        if not self.system_recieved:
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
        if not self.is_PSM_fuckd_voltage or not self.is_PSM_fuckd_current:
            self.system_timer.shutdown()
            self.xavier_timer.shutdown()
            self.log_timer.shutdown()
            self.bus.close()


if __name__ == "__main__":
    bm = BatteryMonitor()
    try:
        rospy.spin()
    finally:
        bm.shutdown()
