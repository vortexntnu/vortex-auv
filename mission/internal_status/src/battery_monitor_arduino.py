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
        # to read voltage and current from Arduino Nano through I2C
        # for code on the arduino:
        self.nano_addr = 12  # I2C adress of nano (setted in software!)
        self.voltage_reg_nano = 0  # value to send to arduino to get voltage read back
        self.current_reg_nano = 1  # to get current measurement back
        self.esc1Current_reg = 2  # to get current measurement from ESC1
        self.esc2Current_reg = 3  # to get current measurement from ESC2
        # init of I2C bus with arduino nano conected
        self.bus = smbus.SMBus(1)
        time.sleep(1)

        # Calibration values for converting from raw digital binary form to decimal form
        # Calibration values were manualy calibrated to +- 0.1V acuracy!
        self.calVoltageA = 11
        self.calVoltageB = 0.0
        # needs to be changed
        self.calCurrent = 37.8788
        self.calCurrentOffset = 0.33

        # Calibration values for converting from the voltage values back to current
        # 11.75 mv = 1 A according to datasheet, not tested
        self.calEscCurrent = 0.01175

        # getting params in the ROS-config file (beluga.yaml)
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
        self.system_current = 0.0
        self.esc1current = 0.0
        self.esc2current = 0.0

        self.system_voltage_state = (
            "No receive"  # should only be "No receive", "Error", "Received"
        )
        self.system_current_state = (
            "No receive"  # should only be "No receive", "Error", "Received"
        )

        # could be used to shut down system when reaching a certain value
        self.I2C_error_counter_voltage = 0

        self.I2C_error_counter_current = 0

        self.system_battery_level_pub = rospy.Publisher(
            "/auv/battery_level/system", Float32, queue_size=1
        )

        self.esc1_current_level_pub = (
            rospy.Publisher(  # Functions for publishing current data to ROS
                "/auv/current_level/ESC1", Float32, queue_size=1
            )
        )

        self.esc2_current_level_pub = rospy.Publisher(
            "/auv/current_level/ESC2", Float32, queue_size=1
        )

        # create current ROS publisher here, if needed

        # set up callbacks
        self.log_timer = rospy.Timer(
            rospy.Duration(secs=logging_interval), self.log_cb
        )  # for logging on ROS terminal

        self.system_timer = rospy.Timer(
            rospy.Duration(secs=system_interval),
            self.system_cb,  # will update and publish measurements to ROS
        )

        rospy.loginfo("BatteryMonitor initialized")

    def system_cb(self, event):
        """Read voltage of system from bootleg ADC."""

        self.read_voltage()
        self.system_battery_level_pub.publish(self.system_voltage)

        self.read_PSM_current()
        # publish current here if needed

        self.read_ESC_current()  # Publishes current from each of the ESCs
        self.esc1_current_level_pub.publish(self.esc1current)
        self.esc2_current_level_pub.publish(self.esc2current)

        if self.system_voltage < self.critical_level:
            rospy.logerr(
                f"Critical voltage: {self.system_voltage}V! Shutting down all active nodes!"
            )
            rospy.logerr(f"HAHA just kidding, let's blow up these batteries!")
            # os.system("rosnode kill -a")

    def log_cb(self, event):
        if self.system_voltage_state == "Received":
            self.log_voltage(self.system_voltage, "system")
        if self.system_voltage_state == "Error":
            rospy.logwarn(
                f"I2C Bus IOerror. Voltage error counter : {self.I2C_error_counter_voltage}"
            )
        if self.system_voltage_state == "No receive":
            rospy.loginfo("No voltage recieved from system yet.")

        # if needed for current, same structure

    def log_voltage(self, voltage, title):
        if voltage == 0:
            rospy.loginfo("Voltage is zero. Killswitch is probably off.")

        elif voltage <= self.warning_level:
            rospy.logwarn("%s voltage: %.3fV" % (title, voltage))

        else:
            rospy.loginfo("%s voltage: %.3fV" % (title, voltage))

    def read_voltage(self):
        # Sometimes an I/O timeout or error happens, it will run again when the error disappears
        try:
            # arduino is configure to send voltage data on "register" 0, current on 1
            # data is sent in 2 bytes, because to big for one I2C message
            voltage_msg = self.bus.read_i2c_block_data(
                self.nano_addr, self.voltage_reg_nano, 2
            )

            # conversion to get real voltage
            # measurement up to 1023, so to big for 7bit I2C messages. Sends MSB first, then LSB, then remap to 0-5V
            x = (((voltage_msg[0] & 0x7) << 7) + voltage_msg[1]) * 5 / 1023.0
            self.system_voltage = x * self.calVoltageA + self.calVoltageB

            # rospy.loginfo(f"Voltage: {self.system_voltage}V")       #for debug

            self.I2C_error_counter_voltage = 0  # no bus error if it reaches that line
            if self.system_voltage_state != "Received":
                self.system_voltage_state = "Received"

        except IOError:
            self.I2C_error_counter_voltage += 1
            self.system_voltage_state = "Error"
            rospy.logwarn(
                f"I2C Bus IOerror. Voltage error counter : {self.I2C_error_counter_voltage}"
            )  # for debug

    def read_PSM_current(self):
        try:
            current_msg = self.bus.read_i2c_block_data(
                self.nano_addr, self.current_reg_nano, 2
            )

            # conversion to get real current
            x = float((((current_msg[0] & 0x7) << 7) + current_msg[1])) * 5 / 1023.0
            self.system_current = (x - self.calCurrentOffset) * self.calCurrent
            # rospy.loginfo(f"Current : {self.system_current}A")

            self.I2C_error_counter_current = 0  # no bus error if it reaches that line
            if self.system_current_state != "Received":
                self.system_current_state = "Received"

        except IOError:
            self.I2C_error_counter_current += 1
            self.system_current_state = "Error"
            # rospy.logwarn(f"I2C Bus IOerror. Voltage error counter : {self.I2C_error_counter_current}")

    def read_ESC_current(self):
        try:
            esc1current_msg = self.bus.read_i2c_block_data(
                self.nano_addr, self.esc1Current_reg, 2
            )

            # conversion to get real current from ESCs
            x = (
                float((((esc1current_msg[0] & 0x7) << 7) + esc1current_msg[1]))
                * 5
                / 1023.0
            )
            self.esc1current = x / self.calEscCurrent

        except IOError:
            pass

        try:
            esc2current_msg = self.bus.read_i2c_block_data(
                self.nano_addr, self.esc2Current_reg, 2
            )

            # conversion to get real current from ESCs
            x = (
                float((((esc2current_msg[0] & 0x7) << 7) + esc2current_msg[1]))
                * 5
                / 1023.0
            )
            self.esc2current = x / self.calEscCurrent

        except IOError:
            pass

    def shutdown(self):
        self.system_timer.shutdown()
        self.log_timer.shutdown()
        self.bus.close()


if __name__ == "__main__":
    bm = BatteryMonitor()
    try:
        rospy.spin()
    finally:
        bm.shutdown()
