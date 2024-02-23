# ROS2 Libraries
import rclpy
from rclpy.node import ROS2Node
from std_msgs.msg import Int16MultiArray

# Custom libraries
from vortex_msgs.msg import ThrusterForces
from thruster_interface_auv_driver_lib import ThrusterInterfaceAUVDriver


class ThrusterInterfaceAUVNode(ROS2Node):

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

        self.get_logger().info(
            '"thruster_interface_auv_node" has been started')

    def _thruster_forces_callback(self, msg):
        # Send thruster forces to be converted into PWM signal and sent to control the thrusters
        # PWM signal gets saved and is published in the "/pwm" topic as a debuging feature to see if everything is alright with the PWM signal
        thruster_forces_array = msg.thrust
        thruster_pwm_array = ThrusterInterfaceAUVDriver.drive_thrusters(
            thruster_forces_array)

        pwm_message = Int16MultiArray()
        pwm_message.data = thruster_pwm_array
        self.thruster_pwm_publisher.publish(pwm_message)


def main(args=None):
    rclpy.init(args=args)

    thruster_interface_auv_node = ThrusterInterfaceAUVNode()
    rclpy.spin(thruster_interface_auv_node)

    thruster_interface_auv_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
