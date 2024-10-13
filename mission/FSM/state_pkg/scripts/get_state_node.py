#!/usr/bin/env python3
import docking_demo
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GetStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("get_state_publisher")
        self.publisher_ = self.create_publisher(String, "current_state", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "go_to_dock"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init()

    get_state_publisher = GetStatePublisher()

    rclpy.spin(get_state_publisher)

    get_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
