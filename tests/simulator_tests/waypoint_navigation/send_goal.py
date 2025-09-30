import random

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import ReferenceFilterWaypoint
from vortex_utils.python_utils import PoseData, euler_to_quat


def randomize_pose() -> PoseData:
    pose: PoseData = PoseData()
    pose.x = random.uniform(-10.0, 10.0)
    pose.y = random.uniform(-10.0, 10.0)
    pose.z = random.uniform(0.5, 3.0)
    pose.roll = 0.0
    # pose.pitch = random.uniform(-1.0, 1.0)
    pose.pitch = random.uniform(0.6, 1.0)
    pose.yaw = random.uniform(-1.57, 1.57)

    return pose


class ReferenceFilterWaypointClient(Node):
    def __init__(self):
        super().__init__('reference_filter_waypoint_client')

        self._action_client = ActionClient(
            self, ReferenceFilterWaypoint, '/orca/reference_filter'
        )
        self.send_goal()

    def send_goal(self):
        goal_pose = randomize_pose()
        goal_msg = ReferenceFilterWaypoint.Goal()

        goal_msg.goal.pose.position.x = goal_pose.x
        goal_msg.goal.pose.position.y = goal_pose.y
        goal_msg.goal.pose.position.z = goal_pose.z
        roll = goal_pose.roll
        pitch = goal_pose.pitch
        yaw = goal_pose.yaw

        quat = euler_to_quat(roll, pitch, yaw)

        goal_msg.goal.pose.orientation.x = quat[0]
        goal_msg.goal.pose.orientation.y = quat[1]
        goal_msg.goal.pose.orientation.z = quat[2]
        goal_msg.goal.pose.orientation.w = quat[3]

        # Write goal pose to temp file
        file_path = "goal_pose.yaml"

        data = {
            "pos": [goal_pose.x, goal_pose.y, goal_pose.z],
            "ori": [goal_pose.roll, goal_pose.pitch, goal_pose.yaw],
        }

        with open(file_path, "w") as f:
            yaml.safe_dump(data, f)

        # Send the goal asynchronously
        self._action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info(f'Sending goal {goal_pose}...')
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


def main(args=None):
    rclpy.init(args=args)
    action_client = ReferenceFilterWaypointClient()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
