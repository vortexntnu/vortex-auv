import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import LOSGuidance


class LOSGuidanceClient(Node):
    def __init__(self):
        super().__init__('los_guidance_client')
        # Create the action client
        self._action_client = ActionClient(self, LOSGuidance, '/orca/los_guidance')
        self.send_goal()

    def send_goal(self):
        goal_msg = LOSGuidance.Goal()

        # Create a PoseStamped message with the goal
        goal_msg.goal.point.x = 20.0
        goal_msg.goal.point.y = 20.0
        goal_msg.goal.point.z = 5.0

        # Send the goal asynchronously
        self._action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result.success
        self.get_logger().info(f'Goal result: {result}')
        if result:   
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
                exit(0)
        else:
            self.get_logger().info('Goal failed :(')
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
                exit(1)


def main(args=None):
    rclpy.init(args=args)
    action_client = LOSGuidanceClient()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
