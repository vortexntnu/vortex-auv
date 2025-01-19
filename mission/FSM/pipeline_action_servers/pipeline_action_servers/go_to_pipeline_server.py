#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node
from vortex_msgs.action import GoToWaypoint
from vortex_msgs.action._go_to_waypoint import GoToWaypoint_Result


class GoToPipelineServer(Node):

    def __init__(self) -> None:
        """Constructor."""
        super().__init__('go_to_pipeline_server')

        self._action_server = ActionServer(self, GoToWaypoint, '/go_to_pipeline', self.execute_callback)

    def execute_callback(self, goal_handle) -> GoToWaypoint_Result:
        """
        This function is called when the action server receives a goal to go down."""
        self.get_logger().info('Executing goal to go to pipeline at: {}'.format(goal_handle.request.waypoint))

        feedback_msg = GoToWaypoint.Feedback()
        docking_position = goal_handle.request.waypoint  # [x, y, z]
        auv_position = PoseStamped()  # Let's say AUV starts at (0,0,0)
        auv_position.pose.position.x = 0.0
        auv_position.pose.position.y = 0.0
        auv_position.pose.position.z = 0.0
        rate = self.create_rate(1)  # Simulate 1Hz feedback

        while (
            math.sqrt(
                (docking_position.pose.position.x - auv_position.pose.position.x) ** 2
                + (docking_position.pose.position.y - auv_position.pose.position.y) ** 2
                + (docking_position.pose.position.z - auv_position.pose.position.z) ** 2
            )
            > 0.1
        ):
            feedback_msg.current_pose = auv_position
            self.get_logger().info(
                'Current position: ({:.2f},{:.2f},{:.2f})'.format(
                    auv_position.pose.position.x, auv_position.pose.position.y, auv_position.pose.position.z
                )
            )

            goal_handle.publish_feedback(feedback_msg)

            # Simulate AUV moving towards the dock
            auv_position.pose.position.x += (docking_position.pose.position.x - auv_position.pose.position.x) / 2
            auv_position.pose.position.y += (docking_position.pose.position.y - auv_position.pose.position.y) / 2
            auv_position.pose.position.z += (docking_position.pose.position.z - auv_position.pose.position.z) / 2

            # rate.sleep()

        goal_handle.succeed()
        result = GoToWaypoint.Result()
        result.success = True
        return result


def main(args=None) -> None:
    """Main function to create and spin the node."""
    rclpy.init(args=args)
    node = GoToPipelineServer()
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
