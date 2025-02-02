#!/usr/bin/env python3
# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Float32MultiArray

# Custom Libraries
import internal_status_auv.gripper_feedback_lib


class GripperFeedbackPublisher(Node):
    def __init__(self):
        # Gripper feedback sensor setup ----------
        self.GripperFeedback = (
            internal_status_auv.gripper_feedback_lib.GripperFeedback()
        )

        # Node setup ----------
        super().__init__("gripper_feedback_publisher")

        # Create publishers ----------
        # Shoulder, wrist, grip
        self.gripper_feedback_publisher = self.create_publisher(
            Float32MultiArray, "/auv/gripper_feedback", 5
        )


def main(args=None):
    rclpy.init(args=args)

    gripper_feedback_publisher = GripperFeedbackPublisher()

    rclpy.spin(gripper_feedback_publisher)

    gripper_feedback_publisher.destroy_node()
    rclpy.shutdown()
