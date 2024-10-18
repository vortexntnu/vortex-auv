#!/usr/bin/env python3
import rclpy
from go_to_dock_action.action import GoToDock
from rclpy.action import ActionClient
from rclpy.node import Node


class GoToDockClient(Node):

    def __init__(self):
        super().__init__('go_to_dock_client')
        self._action_client = ActionClient(self, GoToDock, 'go_to_dock')

    def send_goal(self, docking_position):
        goal_msg = GoToDock.Goal()
        goal_msg.docking_position = docking_position

        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: docking_position={docking_position}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Successfully docked!')
        else:
            self.get_logger().info('Failed to dock.')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(message=f'Received feedback: Distance to dock: {feedback_msg.feedback.distance_to_dock:.2f} meters.')


def main(args=None):
    rclpy.init(args=args)
    node = GoToDockClient()
    docking_position = [10.0, 10.0, 0.0]
    node.send_goal(docking_position)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
