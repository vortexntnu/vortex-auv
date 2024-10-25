#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from vortex_msgs.action import GoToWaypoint
from geometry_msgs.msg import PoseStamped

class ReturnHomeServer(Node):

    def __init__(self):
        super().__init__('return_home_server')

        self._action_server = ActionServer(self, GoToWaypoint, 'return_home', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal to dock at: {}'.format(goal_handle.request.waypoint))

        feedback_msg = GoToWaypoint.Feedback()
        home_position = goal_handle.request.waypoint  # [x, y, z]
        auv_position = PoseStamped()  # Let's say AUV starts at (0,0,0)
        auv_position.pose.position.x = 5.0
        auv_position.pose.position.y = 5.0 
        auv_position.pose.position.z = 10.0
        rate = self.create_rate(1)  # Simulate 1Hz feedback

        while math.sqrt((home_position.pose.position.x - auv_position.pose.position.x)**2 + (home_position.pose.position.y - auv_position.pose.position.y)**2 + (home_position.pose.position.z - auv_position.pose.position.z)**2) > 0.1:
            feedback_msg.current_pose = auv_position
            self.get_logger().info('Current position: ({:.2f},{:.2f},{:.2f})'.format(auv_position.pose.position.x, auv_position.pose.position.y, auv_position.pose.position.z))

            goal_handle.publish_feedback(feedback_msg)

            # Simulate AUV moving towards the dock
            auv_position.pose.position.x += (home_position.pose.position.x- auv_position.pose.position.x)/2
            auv_position.pose.position.y += (home_position.pose.position.y- auv_position.pose.position.y)/2
            auv_position.pose.position.z += (home_position.pose.position.z- auv_position.pose.position.z)/2

            #rate.sleep()

        goal_handle.succeed()
        result = GoToWaypoint.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ReturnHomeServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
