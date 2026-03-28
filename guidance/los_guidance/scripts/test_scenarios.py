import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header
from vortex_msgs.action import LOSGuidance


class WaypointTest(Node):
    def __init__(self):
        super().__init__("waypoint_test_client")

        self.declare_parameter("test_scenario", "4_corner")
        self.declare_parameter("drone", "nautilus")

        self.test_scenario = self.get_parameter("test_scenario").value
        self.drone = self.get_parameter("drone").value

        self._action_client = ActionClient(
            self,
            LOSGuidance,
            f"/{self.drone}/los_guidance",
        )

        self.depth = 2.5
        self.square_size = 10.0

        self.circle_radius = 8.0
        self.circle_points = 16
        self.circle_center_x = 0.0
        self.circle_center_y = 0.0

        self.waypoints = self.generate_waypoints(self.test_scenario)
        self.current_index = 0

        self.get_logger().info(f"Starting test scenario: {self.test_scenario}")
        self.get_logger().info(f"Using drone namespace: {self.drone}")
        self.get_logger().info(f"Number of waypoints: {len(self.waypoints)}")

        self.send_next_goal()

    def generate_waypoints(self, test_scenario):
        if test_scenario == "4_corner":
            s = self.square_size
            d = self.depth
            return [
                (s, 0.0, d),
                (s, s, d),
                (0.0, s, d),
                (0.0, 0.0, d),
            ]

        elif test_scenario == "circle":
            d = self.depth
            waypoints = []

            for i in range(self.circle_points):
                theta = 2.0 * math.pi * i / self.circle_points
                x = self.circle_center_x + self.circle_radius * math.cos(theta)
                y = self.circle_center_y + self.circle_radius * math.sin(theta)
                waypoints.append((x, y, d))

            waypoints.append(waypoints[0])
            return waypoints

        elif test_scenario == "test_pitch":
            # 0 = water surface, do not go above
            # this test scenario has no seabed, so z can be however we need. 
            # Keep all depths safely between these
            return [
                (3.0, 0.0, 1.0),  # slight up
                (6.0, 0.0, 2.0),  # slight down
                (9.0, 0.0, 1.0),  # up again
                (12.0, 0.0, 2.0),  # down again
            ]

        elif test_scenario == "opposite_point":
            # Go to one point, then the exact opposite point
            return [
                (6.0, 4.0, self.depth),
                (-6.0, -4.0, self.depth),
            ]

        else:
            self.get_logger().warn(
                f"Unknown test_scenario '{test_scenario}', defaulting to 4_corner"
            )
            return self.generate_waypoints("4_corner")

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info(f"{self.test_scenario} test completed!")
            rclpy.shutdown()
            return

        self._action_client.wait_for_server()

        goal_msg = LOSGuidance.Goal()

        header = Header()
        header.frame_id = "world_ned"
        goal_msg.goal.header = header

        x, y, z = self.waypoints[self.current_index]
        goal_msg.goal.point.x = float(x)
        goal_msg.goal.point.y = float(y)
        goal_msg.goal.point.z = float(z)

        self.get_logger().info(
            f"Sending waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}, z={z:.2f}"
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

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
    node = WaypointTest()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
