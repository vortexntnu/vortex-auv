#!/usr/bin/env python3
# ROS2 Libraries
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from thruster_interface_auv.thruster_interface_auv_driver_lib import (
    ThrusterInterfaceAUVDriver,
)

# Custom libraries
from vortex_msgs.msg import ThrusterForces


class ThrusterInterfaceAUVNode(Node):
    def __init__(self) -> None:
        # Initialize and name the node process running
        super().__init__("thruster_interface_auv_node")

        # Create a subscriber that takes data from thruster forces
        # Then convert this Forces into PWM signals and control the thrusters
        # Publish PWM values as deebuging feature
        self.thruster_forces_subscriber = self.create_subscription(ThrusterForces, "thrust/thruster_forces", self._thruster_forces_callback, 10)
        self.thruster_pwm_publisher = self.create_publisher(Int16MultiArray, "pwm_APPROX", 10)

        # Get thruster mapping, direction, offset and clamping parameters
        self.declare_parameter("propulsion.thrusters.thruster_to_pin_mapping", [7, 6, 5, 4, 3, 2, 1, 0])
        self.declare_parameter("propulsion.thrusters.thruster_direction", [1, 1, 1, 1, 1, 1, 1, 1])
        self.declare_parameter("propulsion.thrusters.thruster_PWM_offset", [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter(
            "propulsion.thrusters.thruster_PWM_min",
            [1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100],
        )
        self.declare_parameter(
            "propulsion.thrusters.thruster_PWM_max",
            [1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900],
        )

        self.declare_parameter("propulsion.thrusters.thrust_update_rate", 10.0)

        self.declare_parameter("coeffs.10V.LEFT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.10V.RIGHT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.12V.LEFT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.12V.RIGHT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.14V.LEFT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.14V.RIGHT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.16V.LEFT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.16V.RIGHT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.18V.LEFT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.18V.RIGHT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.20V.LEFT", None, ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("coeffs.20V.RIGHT", None, ParameterDescriptor(dynamic_typing=True))

        self.thruster_mapping = self.get_parameter("propulsion.thrusters.thruster_to_pin_mapping").value
        self.thruster_direction = self.get_parameter("propulsion.thrusters.thruster_direction").value
        self.thruster_PWM_offset = self.get_parameter("propulsion.thrusters.thruster_PWM_offset").value
        self.thruster_PWM_min = self.get_parameter("propulsion.thrusters.thruster_PWM_min").value
        self.thruster_PWM_max = self.get_parameter("propulsion.thrusters.thruster_PWM_max").value
        self.thrust_timer_period = 1.0 / self.get_parameter("propulsion.thrusters.thrust_update_rate").value

        # dictionary for the coeffs of the poly approx of thruster forces based on OPERATIONAL_VOLTAGE
        coeffs = {
            10: {
                "LEFT": self.get_parameter("coeffs.10V.LEFT").value,
                "RIGHT": self.get_parameter("coeffs.10V.RIGHT").value,
            },
            12: {
                "LEFT": self.get_parameter("coeffs.12V.LEFT").value,
                "RIGHT": self.get_parameter("coeffs.12V.RIGHT").value,
            },
            14: {
                "LEFT": self.get_parameter("coeffs.14V.LEFT").value,
                "RIGHT": self.get_parameter("coeffs.14V.RIGHT").value,
            },
            16: {
                "LEFT": self.get_parameter("coeffs.16V.LEFT").value,
                "RIGHT": self.get_parameter("coeffs.16V.RIGHT").value,
            },
            18: {
                "LEFT": self.get_parameter("coeffs.18V.LEFT").value,
                "RIGHT": self.get_parameter("coeffs.18V.RIGHT").value,
            },
            20: {
                "LEFT": self.get_parameter("coeffs.20V.LEFT").value,
                "RIGHT": self.get_parameter("coeffs.20V.RIGHT").value,
            },
        }

        print(coeffs)

        # Initialize thruster driver
        self.thruster_driver = ThrusterInterfaceAUVDriver(
            ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET=get_package_share_directory("thruster_interface_auv"),
            THRUSTER_MAPPING=self.thruster_mapping,
            THRUSTER_DIRECTION=self.thruster_direction,
            THRUSTER_PWM_OFFSET=self.thruster_PWM_offset,
            PWM_MIN=self.thruster_PWM_min,
            PWM_MAX=self.thruster_PWM_max,
            coeffs=coeffs,
        )

        # Start clock timer for driving thrusters every 0.2 seconds
        # Declare "self.thruster_forces_array" in case no topic comes in at the first possible second
        self.thruster_forces_array = [2.23] * 8

        self.timer = self.create_timer(self.thrust_timer_period, self._timer_callback)

        # Debugging
        self.get_logger().info('"thruster_interface_auv_node" has been started')

    def _thruster_forces_callback(self, msg) -> None:
        # Get data of the forces published
        self.thruster_forces_array = msg.thrust

    def _timer_callback(self) -> None:
        # Send thruster forces to be converted into PWM signal and sent to control the thrusters
        # PWM signal gets saved and is published in the "/pwm" topic as a debugging feature to see if everything is alright with the PWM signal
        thruster_pwm_array = self.thruster_driver.drive_thrusters(self.thruster_forces_array)

        pwm_message = Int16MultiArray()
        pwm_message.data = thruster_pwm_array
        self.thruster_pwm_publisher.publish(pwm_message)


def main(args=None) -> None:
    # Initialize
    rclpy.init(args=args)

    # Running
    thruster_interface_auv_node = ThrusterInterfaceAUVNode()
    rclpy.spin(thruster_interface_auv_node)

    # Shutdown
    thruster_interface_auv_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
