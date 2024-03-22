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

        # Get thruster mapping, direction and offset parameters
        self.declare_parameter('propulsion.thrusters.map',
                               [7, 6, 5, 4, 3, 2, 1, 0])
        self.declare_parameter('propulsion.thrusters.direction',
                               [1, 1, 1, 1, 1, 1, 1, 1])
        self.declare_parameter('propulsion.thrusters.offset',
                               [80, 80, 80, 80, 80, 80, 80, 80])

        self.thruster_mapping = self.get_parameter(
            'propulsion.thrusters.map').value
        self.thruster_direction = self.get_parameter(
            'propulsion.thrusters.direction').value
        self.thruster_offset = self.get_parameter(
            'propulsion.thrusters.offset').value

        # Initialize thruster driver
        self.thruster_driver = ThrusterInterfaceAUVDriver(
            ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET=
            get_package_share_directory("thruster_interface_auv"),
            THRUSTER_MAPPING=self.thruster_mapping,
            THRUSTER_DIRECTION=self.thruster_direction,
            THRUSTER_OFFSET=self.thruster_offset)

        self.get_logger().info(
            '"thruster_interface_auv_node" has been started')

    def _thruster_forces_callback(self, msg):
        # Send thruster forces to be converted into PWM signal and sent to control the thrusters
        # PWM signal gets saved and is published in the "/pwm" topic as a debuging feature to see if everything is alright with the PWM signal
        thruster_forces_array = msg.thrust
        thruster_pwm_array = self.thruster_driver.drive_thrusters(
            thruster_forces_array)

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
