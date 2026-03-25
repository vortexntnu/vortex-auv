import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header
from vortex_msgs.action import LOSGuidance


class LOSGuidanceClient(Node):
    def __init__(self):
        super().__init__("los_guidance_client")

        self.declare_parameter("drone", "orca")
        self.declare_parameter("x", 20.0)
        self.declare_parameter("y", 20.0)
        self.declare_parameter("z", 2.5)

        self.drone = self.get_parameter("drone").value
        self.goal_x = float(self.get_parameter("x").value)
        self.goal_y = float(self.get_parameter("y").value)
        self.goal_z = float(self.get_parameter("z").value)

        self._action_client = ActionClient(
            self,
            LOSGuidance,
            f"/{self.drone}/los_guidance",
        )

        self.get_logger().info(f"Using drone namespace: {self.drone}")
        self.send_goal()

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available")
            self.shutdown_with_code(1)
            return

        goal_msg = LOSGuidance.Goal()

        header = Header()
        header.frame_id = "world_ned"
        header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.header = header

        goal_msg.goal.point.x = self.goal_x
        goal_msg.goal.point.y = self.goal_y
        goal_msg.goal.point.z = self.goal_z

        self.get_logger().info(
            f"Sending goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, z={self.goal_z:.2f}"
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
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
