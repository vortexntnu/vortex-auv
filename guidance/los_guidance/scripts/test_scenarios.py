import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import LOSWaypoint
from vortex_msgs.msg import LOSWaypoint as LOSWaypointMsg


class WaypointTest(Node):
    def __init__(self):
        super().__init__("waypoint_test_client")

        self.declare_parameter("test_scenario", "4_corner")
        self.declare_parameter("drone", "nautilus")

        self.test_scenario = self.get_parameter("test_scenario").value
        self.drone = self.get_parameter("drone").value

        self._action_client = ActionClient(
            self,
            LOSWaypoint,
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
            return [
                (3.0, 0.0, 1.0),
                (6.0, 0.0, 2.0),
                (9.0, 0.0, 1.0),
                (12.0, 0.0, 2.0),
            ]

        elif test_scenario == "opposite_point":
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

        x, y, z = self.waypoints[self.current_index]

        los_wp = LOSWaypointMsg()
        los_wp.waypoints = Point(x=float(x), y=float(y), z=float(z))

        goal_msg = LOSWaypoint.Goal()
        goal_msg.los_waypoint = los_wp
        goal_msg.convergence_threshold = 0.3

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
