import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from vortex_msgs.action import LOSGuidance
from geometry_msgs.msg import Point
from std_msgs.msg import Header


class SquareTest(Node):

    def __init__(self):
        super().__init__('square_test_client')

        self._action_client = ActionClient(
            self,
            LOSGuidance,
            '/orca/los_guidance'
        )

        self.depth = 2.5   
        self.size = 10.0

        self.waypoints = [
            (self.size, 0.0, self.depth),
            (self.size, self.size, self.depth),
            (0.0, self.size, self.depth),
            (0.0, 0.0, self.depth),
        ]

        self.current_index = 0

        self.send_next_goal()

    def send_next_goal(self):

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("Square test completed!")
            rclpy.shutdown()
            return

        self._action_client.wait_for_server()

        goal_msg = LOSGuidance.Goal()

        header = Header()
        header.frame_id = "world_ned"

        goal_msg.goal.header = header

        x, y, z = self.waypoints[self.current_index]

        goal_msg.goal.point.x = x
        goal_msg.goal.point.y = y
        goal_msg.goal.point.z = z

        self.get_logger().info(
            f"Sending waypoint {self.current_index + 1}: "
            f"x={x}, y={y}, z={z}"
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):

        self.get_logger().info("Waypoint reached")

        self.current_index += 1
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SquareTest()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
