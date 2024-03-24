#!/usr/bin/env python3
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from ament_index_python.packages import get_package_share_directory

# Custom libraries
from vortex_msgs.msg import ThrusterForces
from thruster_interface_auv.thruster_interface_auv_driver_lib import ThrusterInterfaceAUVDriver


class ThrusterInterfaceAUVNode(Node):

    def __init__(self):
        # Initialize and name the node process running
        super().__init__("thruster_interface_auv_node")

        # Create a subscriber that takes data from thruster forces
        # Then convert this Forces into PWM signals and control the thrusters
        # Publish PWM values as deebuging feature
        self.thruster_forces_subscriber = self.create_subscription(
            ThrusterForces, "thrust/thruster_forces",
            self._thruster_forces_callback, 10)
        self.thruster_pwm_publisher = self.create_publisher(
            Int16MultiArray, 'pwm', 10)

        # Get thruster mapping, direction, offset and clamping parameters
        self.declare_parameter('propulsion.thrusters.thruster_to_pin_mapping',
                               [7, 6, 5, 4, 3, 2, 1, 0])
        self.declare_parameter('propulsion.thrusters.thruster_direction',
                               [1, 1, 1, 1, 1, 1, 1, 1])
        self.declare_parameter('propulsion.thrusters.thruster_PWM_offset',
                               [0, 0, 0, 0, 0, 0, 0, 0])
        self.declare_parameter(
            'propulsion.thrusters.thruster_PWM_min',
            [1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100])
        self.declare_parameter(
            'propulsion.thrusters.thruster_PWM_max',
            [1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900])

        self.thruster_mapping = self.get_parameter(
            'propulsion.thrusters.thruster_to_pin_mapping').value
        self.thruster_direction = self.get_parameter(
            'propulsion.thrusters.thruster_direction').value
        self.thruster_PWM_offset = self.get_parameter(
            'propulsion.thrusters.thruster_PWM_offset').value
        self.thruster_PWM_min = self.get_parameter(
            'propulsion.thrusters.thruster_PWM_min').value
        self.thruster_PWM_max = self.get_parameter(
            'propulsion.thrusters.thruster_PWM_max').value

        # Initialize thruster driver
        self.thruster_driver = ThrusterInterfaceAUVDriver(
            ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET=
            get_package_share_directory("thruster_interface_auv"),
            THRUSTER_MAPPING=self.thruster_mapping,
            THRUSTER_DIRECTION=self.thruster_direction,
            THRUSTER_PWM_OFFSET=self.thruster_PWM_offset,
            PWM_MIN=self.thruster_PWM_min,
            PWM_MAX=self.thruster_PWM_max)

        # Start clock timer for driving thrusters every 0.2 seconds
        # Declare "self.thruster_forces_array" in case no topic comes in at the first possible second
        self.thruster_forces_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.timer = self.create_timer(0.2, self._timer_callback)

        # Debugging
        self.get_logger().info(
            '"thruster_interface_auv_node" has been started')

    def _thruster_forces_callback(self, msg):
        # Get data of the forces published
        self.thruster_forces_array = msg.thrust

    def _timer_callback(self):
        # Send thruster forces to be converted into PWM signal and sent to control the thrusters
        # PWM signal gets saved and is published in the "/pwm" topic as a debuging feature to see if everything is alright with the PWM signal
        thruster_pwm_array = self.thruster_driver.drive_thrusters(
            self.thruster_forces_array)

        pwm_message = Int16MultiArray()
        pwm_message.data = thruster_pwm_array
        self.thruster_pwm_publisher.publish(pwm_message)


def main(args=None):
    # Initialize
    rclpy.init(args=args)

    # Running
    thruster_interface_auv_node = ThrusterInterfaceAUVNode()
    rclpy.spin(thruster_interface_auv_node)

    # Shutdown
    thruster_interface_auv_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
