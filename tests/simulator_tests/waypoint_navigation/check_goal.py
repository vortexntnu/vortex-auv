import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

best_effort_qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

goal = (2.0, 4.5, 3.0)
tolerance = 0.3  # meters


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

    print(f"Waiting for drone to reach goal: {goal}")

    start_time = time.time()
    timeout = 20  # seconds

    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(node)
        if node.received_pose_:
            x = node.current_pose_.pose.pose.position.x
            y = node.current_pose_.pose.pose.position.y
            z = node.current_pose_.pose.pose.position.z
            dist = math.sqrt(
                (goal[0] - x) ** 2 + (goal[1] - y) ** 2 + (goal[2] - z) ** 2
            )

            if dist < tolerance:
                print(f"Drone reached goal: {goal}")
                rclpy.shutdown()
                exit(0)

    print(f"Drone did not reach goal: {goal}")
    rclpy.shutdown()
    exit(1)


if __name__ == '__main__':
    main()
