import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import GuidanceWaypoint
from vortex_msgs.msg import Waypoint, WaypointMode


class LOSGuidanceClient(Node):
    def __init__(self):
        super().__init__('los_guidance_client')

        self._action_client = ActionClient(
            self, GuidanceWaypoint, '/nautilus/los_guidance'
        )
        self.send_goal()

    def send_goal(self):
        goal_msg = GuidanceWaypoint.Goal()

        pose = Pose()
        pose.position.x = 20.0
        pose.position.y = 20.0
        pose.position.z = 5.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        waypoint = Waypoint()
        waypoint.pose = pose

        waypoint_mode = WaypointMode()
        waypoint_mode.mode = WaypointMode.ONLY_POSITION
        waypoint.waypoint_mode = waypoint_mode

        goal_msg.waypoint = waypoint
        goal_msg.convergence_threshold = 0.5

        self._action_client.wait_for_server(timeout_sec=10.0)

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error("Failed to send goal")
            self.shutdown_with_code(1)
            return

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.shutdown_with_code(1)
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result_msg = future.result()

        if result_msg is None:
            self.get_logger().error("Did not receive result")
            self.shutdown_with_code(1)
            return

        result = result_msg.result
        status = result_msg.status

        self.get_logger().info(f"Result status: {status}")
        self.get_logger().info(f"Goal success: {result.success}")

        if result.success:
            self.get_logger().info("Goal reached successfully")
            self.shutdown_with_code(0)
        else:
            self.get_logger().error("Goal failed")
            self.shutdown_with_code(1)

    def feedback_callback(self, feedback_msg):
        self.get_logger().debug("Received feedback")

    def shutdown_with_code(self, code):
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LOSGuidanceClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
