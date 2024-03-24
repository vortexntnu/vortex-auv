#!/usr/bin/env python3
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Float32

# Custom Libraries
import internal_status_auv.power_sense_module_lib


class PowerSenseModulePublisher(Node):

    def __init__(self):
        # Node setup ----------
        super().__init__('power_sense_module_publisher')
        self.PSM = internal_status_auv.power_sense_module_lib.PowerSenseModule(
        )

        # Create publishers ----------
        self.publisher_current = self.create_publisher(
            Float32, '/auv/power_sense_module/current', 5)
        self.publisher_voltage = self.create_publisher(
            Float32, '/auv/power_sense_module/voltage', 5)

        # Data gathering cycle ----------
        self.current = 0.0
        self.voltage = 0.0

        self.declare_parameter(
            "internal_status.power_sense_module_read_rate", 10.0
        )  # Providing a default value 10.0 => 0.1 second delay per data gathering
        read_rate = self.get_parameter(
            "internal_status.power_sense_module_read_rate"
        ).get_parameter_value().double_value
        read_timer_period = 1.0 / read_rate
        self.read_timer = self.create_timer(read_timer_period,
                                            self.read_timer_callback)

        # Watchdog for anomalies ----------
        self.logger = get_logger("power_sense_module")

        self.declare_parameter("internal_status.voltage_min", 14.5)
        self.voltageMin = self.get_parameter(
            "internal_status.voltage_min").get_parameter_value().double_value

        self.declare_parameter("internal_status.voltage_max", 16.8)
        self.voltageMax = self.get_parameter(
            "internal_status.voltage_max").get_parameter_value().double_value

        self.declare_parameter(
            "internal_status.power_sense_module_warning_rate", 0.1)
        warning_rate = self.get_parameter(
            "internal_status.power_sense_module_warning_rate"
        ).get_parameter_value().double_value
        warning_timer_period = 1.0 / warning_rate
        self.warning_timer = self.create_timer(warning_timer_period,
                                               self.warning_timer_callback)

        # Debuging ----------
        self.get_logger().info(
            '"power_sense_module_publisher" has been started')

    def read_timer_callback(self):
        # Get the PSM data
        self.current = self.PSM.get_current()
        self.voltage = self.PSM.get_voltage()

        # Publish PSM data
        current_msg = Float32()
        voltage_msg = Float32()

        current_msg.data = self.current
        voltage_msg.data = self.voltage

        self.publisher_current.publish(
            current_msg)  #publish current value to the "current topic"
        self.publisher_voltage.publish(
            voltage_msg)  #publish voltage value to the "voltge topic"

    def warning_timer_callback(self):
        # Check if Voltage is abnormal and if so print a warning
        if (self.voltage < self.voltageMin):
            self.logger.fatal(
                f"WARNING: Battery Voltage to LOW at {self.voltage} V")
        elif (self.voltage > self.voltageMax):
            self.logger.fatal(
                f"WARNING: Battery Voltage to HIGH at {self.voltage} V")


def main(args=None):
    rclpy.init(args=args)

    power_sense_module_publisher = PowerSenseModulePublisher()

    rclpy.spin(power_sense_module_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    power_sense_module_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
