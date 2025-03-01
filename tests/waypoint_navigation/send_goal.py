import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import ReferenceFilterWaypoint


class ReferenceFilterWaypointClient(Node):
    def __init__(self):
        super().__init__('reference_filter_waypoint_client')
        # Create the action client
        self._action_client = ActionClient(
            self, ReferenceFilterWaypoint, '/reference_filter'
        )
        self.send_goal()

    def send_goal(self):
        goal_msg = ReferenceFilterWaypoint.Goal()

        # Create a PoseStamped message with the goal
        goal_msg.goal.pose.position.x = 2.0
        goal_msg.goal.pose.position.y = 4.5
        goal_msg.goal.pose.position.z = 3.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        quat = self.euler_to_quat(roll, pitch, yaw)

        goal_msg.goal.pose.orientation.w = quat[0]
        goal_msg.goal.pose.orientation.x = quat[1]
        goal_msg.goal.pose.orientation.y = quat[2]
        goal_msg.goal.pose.orientation.z = quat[3]

        # Send the goal asynchronously
        self._action_client.wait_for_server()
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
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    @staticmethod
    def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return np.array([w, x, y, z])


def main(args=None):
    rclpy.init(args=args)
    action_client = ReferenceFilterWaypointClient()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
