#!/usr/bin/env python3
import math

import rclpy
from go_to_dock_action.action import FindDock  # Import the action definition
from rclpy.action import ActionServer
from rclpy.node import Node



class FindDockServer(Node):

    def __init__(self):
        super().__init__('go_to_dock_server')

        self._action_server = ActionServer(self, FindDock, 'find_dock', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal to dock at: {}'.format(goal_handle.request.found))

        feedback_msg = FindDock.Feedback()
        found = goal_handle.request.found  # bool
        rate = self.create_rate(1)  # Simulate 1Hz feedback
        num_steps = 0

        while not num_steps == 1:
            num_steps += 1
            feedback_msg.num_checked_waypoints = num_steps
            self.get_logger().info('Num waypoints searched: {:.2f}'.format(num_steps))

            goal_handle.publish_feedback(feedback_msg)

            #rate.sleep()

        goal_handle.succeed()
        result = FindDock.Result()
        result.docking_station_location = [5.0, 5.0, 10.0]
        return result




def main(args=None):
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
