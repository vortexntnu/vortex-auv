#!/usr/bin/python3
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Float32

# Custom Libraries
import internal_status_auv.temperature_sensor_lib


class TemperaturePublisher(Node):
    def __init__(self):
        # Pressure sensor setup ----------
        self.Temperature = (
            internal_status_auv.temperature_sensor_lib.TemperatureSensor()
        )

        # Node setup ----------
        super().__init__("temperature_sensor_publisher")

        # Create publishers ----------
        self.publisher_temperature = self.create_publisher(
            Float32, "/auv/temperature", 5
        )

        # Data gathering cycle ----------
        self.temperature = 0.0

        self.declare_parameter(
            "internal_status.temperature_read_rate", 0.1
        )  # Providing a default value 0.1 => 10 second delay per data gathering
        read_rate = (
            self.get_parameter("internal_status.temperature_read_rate")
            .get_parameter_value()
            .double_value
        )
        timer_period = 1.0 / read_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Watchdog for anomalies ----------
        self.logger = get_logger("temperature_sensor")

        self.declare_parameter("internal_status.temperature_critical_level", 90.0)
        self.temperatureCriticalLevel = (
            self.get_parameter("internal_status.temperature_critical_level")
            .get_parameter_value()
            .double_value
        )

        self.declare_parameter("internal_status.temperature_warning_rate", 0.1)
        warning_rate = (
            self.get_parameter("internal_status.temperature_warning_rate")
            .get_parameter_value()
            .double_value
        )
        warning_timer_period = 1.0 / warning_rate
        self.warning_timer = self.create_timer(
            warning_timer_period, self.warning_timer_callback
        )

        # Debuging ----------
        self.get_logger().info('"temperature_sensor_publisher" has been started')

    def timer_callback(self):
        # Get temperature data
        self.temperature = self.Temperature.get_temperature()

        # Publish temperature data
        temperature_msg = Float32()
        temperature_msg.data = self.temperature
        self.publisher_temperature.publish(temperature_msg)

    def warning_timer_callback(self):
        # Check if Temperature is abnormal and if so print a warning
        if self.temperature > self.temperatureCriticalLevel:
            self.logger.fatal(
                f"WARNING: Temperature inside the Drone to HIGH: {self.temperature} *C! Drone might be overheating!"
            )


def main(args=None):
    rclpy.init(args=args)

    temperature_publisher = TemperaturePublisher()

    rclpy.spin(temperature_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    temperature_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
