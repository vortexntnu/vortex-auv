#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node
from vortex_msgs.action import FindDock
from vortex_msgs.action._find_dock import (
    FindDock_Result,  # Import the action definition
)


class FindDockServer(Node):

    def __init__(self) -> None:
        """
        Initialize the action server for finding the dock."""
        super().__init__('find_dock_server')

        self._action_server = ActionServer(self, FindDock, 'find_dock', self.execute_callback)

    def execute_callback(self, goal_handle) -> FindDock_Result:
        """
        This function is called when the action server receives a goal to find the dock."""
        self.get_logger().info('Executing goal to dock at: {}'.format(goal_handle.request.start_search))

        feedback_msg = FindDock.Feedback()
        found = goal_handle.request.start_search  # bool
        rate = self.create_rate(1)  # Simulate 1Hz feedback
        time_elapsed = 0.0

        while time_elapsed <= 20.0:
            feedback_msg.time_elapsed = time_elapsed
            self.get_logger().info('Time elapsed: {:.2f}'.format(time_elapsed))

            goal_handle.publish_feedback(feedback_msg)

            # rate.sleep()
            time_elapsed += 0.1

        goal_handle.succeed()
        result = FindDock.Result()

        result.dock_pose = PoseStamped()
        result.dock_pose.pose.position.x = 5.0
        result.dock_pose.pose.position.y = 5.0
        result.dock_pose.pose.position.z = 10.0
        result.dock_pose.pose.orientation.x = 0.0
        result.dock_pose.pose.orientation.y = 0.0
        result.dock_pose.pose.orientation.z = 0.0
        result.dock_pose.pose.orientation.w = 0.0
        result.dock_pose.header.stamp = self.get_clock().now().to_msg()
        result.dock_pose.header.frame_id = '0'

        return result


def main(args=None) -> None:
    """
    Main function to run the FindDockServer node."""
    rclpy.init(args=args)
    node = FindDockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
