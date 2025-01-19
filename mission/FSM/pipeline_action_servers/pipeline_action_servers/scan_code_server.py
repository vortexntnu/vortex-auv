#!/usr/bin/env python3
import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node
from vortex_msgs.action import ScanCode


class ScanCodeServer(Node):
    
    def __init__(self) -> None:
        """
        Initialize the action server for scanning the aruco code."""
        super().__init__('scan_code_server')

        self._action_server = ActionServer(self, ScanCode, 'scan_code', self.execute_callback)

    def execute_callback(self, goal_handle) -> ScanCode.Result:
        """
        This function is called when the action server receives a goal to scan the aruco code.
        For testing purposes, the server will return a dummy result. It should be a random id from the aruco marker."""

        self.get_logger().info('Executing goal to scan aruco code at: {}'.format(goal_handle.request.start_search))

        feedback_msg = ScanCode.Feedback()
        found = goal_handle.request.start_search
        rate = self.create_rate(1)
        time_elapsed = 0.0

        while time_elapsed <= 20.0:
            feedback_msg.time_elapsed = time_elapsed
            self.get_logger().info('Time elapsed: {:.2f}'.format(time_elapsed))

            goal_handle.publish_feedback(feedback_msg)

            time_elapsed += 0.1
        
        goal_handle.succeed()
        result = ScanCode.Result()

        result.aruco_id = random.randint(0, 100)



        
        return result
    

def main(args=None) -> None:
    """
    Main function to start the ScanCodeServer node."""
    rclpy.init(args=args)

    scan_code_server = ScanCodeServer()

    try:
        rclpy.spin(scan_code_server)
    except KeyboardInterrupt:
        pass

    scan_code_server.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()