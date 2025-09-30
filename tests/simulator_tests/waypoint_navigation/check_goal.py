import math
import os
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from vortex_utils.python_utils import quat_to_euler

best_effort_qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

# Read goal from temp file
file_path = "goal_pose.yaml"
with open(file_path) as f:
    data = yaml.safe_load(f)

# Remove temp file
os.remove(file_path)
print(f"Temp file {file_path} deleted")
goal_pos = data["pos"]
goal_ori = data["ori"]

pos_tol = 0.3  # meters
ori_tol = 0.3  # rad


class CheckGoalNode(Node):
    def __init__(self):
        super().__init__('check_goal_node')
        self.pose_sub_ = self.create_subscription(
            PoseWithCovarianceStamped, '/orca/pose', self.pose_callback, best_effort_qos
        )

        self.current_pose_: PoseWithCovarianceStamped = None
        self.received_pose_: bool = False

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose_ = msg
        self.received_pose_ = True


def main(args=None):
    rclpy.init(args=args)
    node = CheckGoalNode()

    print(f"Waiting for drone to reach goal: {goal_pos} with orientation {goal_ori}")

    start_time = time.time()
    timeout = 20  # seconds

    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(node)
        if node.received_pose_:
            x = node.current_pose_.pose.pose.position.x
            y = node.current_pose_.pose.pose.position.y
            z = node.current_pose_.pose.pose.position.z
            q_w = node.current_pose_.pose.pose.orientation.w
            q_x = node.current_pose_.pose.pose.orientation.x
            q_y = node.current_pose_.pose.pose.orientation.y
            q_z = node.current_pose_.pose.pose.orientation.z
            current_ori = quat_to_euler(q_x, q_y, q_z, q_w)
            dist = math.sqrt(
                (goal_pos[0] - x) ** 2 + (goal_pos[1] - y) ** 2 + (goal_pos[2] - z) ** 2
            )

            dist_ori = math.sqrt(
                (goal_ori[0] - current_ori[0]) ** 2
                + (goal_ori[1] - current_ori[1]) ** 2
                + (goal_ori[2] - current_ori[2]) ** 2
            )

            if dist < pos_tol and dist_ori < ori_tol:
                print(f"Drone reached goal: {goal_pos} and orientation: {goal_ori}")
                rclpy.shutdown()
                exit(0)

    print(f"Drone did not reach goal: {goal_pos} and orientation: {goal_ori}")
    print(f"Current_drone pose: ({x, y, z}), {current_ori}")
    rclpy.shutdown()
    exit(1)


if __name__ == '__main__':
    main()
