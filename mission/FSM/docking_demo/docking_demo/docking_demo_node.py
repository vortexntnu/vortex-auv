import rclpy
from rclpy.node import Node


class DockingDemoNode(Node):
    def __init__(self) -> None:
        """Constructor."""
        super().__init__('docking_demo_node')
        self.get_logger().info('Docking Demo Node has been started')
        self.create_timer(1, self.timer_callback)

    def timer_callback(self) -> None:
        """Timer callback function."""
        self.get_logger().info('Docking Demo Node is running')


def main(args: None = None) -> None:
    """Entry point for the docking_demo_node executable."""
    rclpy.init(args=args)
    node = DockingDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()
