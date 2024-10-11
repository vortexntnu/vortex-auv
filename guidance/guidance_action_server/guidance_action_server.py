import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from guidance_action_server.action import NavigateWaypoints
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from vortex_msgs.msg import LOSGuidance
import numpy as np
import math
from transforms3d.euler import quat2euler
import time

def normalize_angle(angle):
    """Normalize angle to be within [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class GuidanceActionServer(Node):

    def __init__(self):
        super().__init__('guidance_action_server')

        # Publishers
        self.output_pub = self.create_publisher(LOSGuidance, '/guidance/los', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '/guidance/reference', 10)
        self.error_pub = self.create_publisher(Vector3Stamped, '/guidance/errors', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/guidance/twist', 10)

        # Subscriber to odometry
        self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            self.execute_callback)

        # Initialize variables
        self.current_position = None
        self.current_velocity = None
        self.current_orientation = None

        # Parameters for guidance algorithm
        self.U = 1.0    # Desired surge speed (m/s)
        self.delta = 0.2  # Lookahead distance (m)

        # Waypoints
        self.waypoints = []
        self.current_waypoint_index = 0

        # Starting point
        self.starting_point = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def odom_callback(self, msg):
        """Callback to handle odometry updates."""
        self.current_position = msg.pose.pose
        self.current_velocity = msg.twist.twist
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        roll, pitch, yaw = quat2euler(quaternion)
        self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        waypoints_msg = goal_handle.request.waypoints
        self.waypoints = [{'x': wp.position.x, 'y': wp.position.y, 'z': wp.position.z} for wp in waypoints_msg]
        self.current_waypoint_index = 0

        success = True

        while self.current_waypoint_index < len(self.waypoints):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                goal_handle.canceled()
                return NavigateWaypoints.Result(success=False)

            # Check if current position and orientation are available
            if self.current_position is None or self.current_orientation is None:
                time.sleep(0.1)
                continue

            waypoint = self.waypoints[self.current_waypoint_index]

            # Extract current position and orientation
            current_x = self.current_position.position.x
            current_y = self.current_position.position.y
            current_z = self.current_position.position.z
            psi = self.current_orientation['yaw']

            # Extract target waypoint position
            target_x = waypoint['x']
            target_y = waypoint['y']
            target_z = waypoint['z']

            # Compute distance to the waypoint
            distance = math.sqrt((target_x - current_x) ** 2 +
                                 (target_y - current_y) ** 2 +
                                 (target_z - current_z) ** 2)

            # Check if waypoint is reached
            if distance < 0.1:
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.get_logger().info('All waypoints reached.')
                    break
                else:
                    waypoint = self.waypoints[self.current_waypoint_index]
                    target_x = waypoint['x']
                    target_y = waypoint['y']
                    target_z = waypoint['z']

            # Guidance calculations using LOS algorithm
            # Reference point is either the starting point or the previous waypoint
            if self.current_waypoint_index == 0:
                ref_x = self.starting_point['x']
                ref_y = self.starting_point['y']
                ref_z = self.starting_point['z']
            else:
                prev_waypoint = self.waypoints[self.current_waypoint_index - 1]
                ref_x = prev_waypoint['x']
                ref_y = prev_waypoint['y']
                ref_z = prev_waypoint['z']

            # Differences between target and reference point
            dx = target_x - ref_x
            dy = target_y - ref_y
            dz = target_z - ref_z

            # Desired path angle
            pi_p = math.atan2(dy, dx)
            pi_p = normalize_angle(pi_p)

            # Compute pitch angle (gamma)
            gamma = math.atan2(-dz, math.hypot(dx, dy))

            # Compute along-track and cross-track errors
            # Transform position error into path coordinate frame
            R = np.array([[math.cos(pi_p), math.sin(pi_p)],
                          [-math.sin(pi_p), math.cos(pi_p)]])
            e_pos = np.array([current_x - ref_x, current_y - ref_y])
            eps = R @ e_pos

            e_x = eps[0]  # Along-track error
            e_y = eps[1]  # Cross-track error

            # Compute desired course angle
            course_d = pi_p - math.atan2(e_y, self.delta)
            course_d = normalize_angle(course_d)

            # Heading error computation
            heading_d = course_d
            e_psi = normalize_angle(heading_d - psi)

            # Set desired surge velocity (could be adjusted based on e_x)
            surge_velocity = self.U

            # Prepare and publish the output message using LOSGuidance
            output_msg = LOSGuidance()
            output_msg.surge = surge_velocity
            output_msg.pitch = gamma
            output_msg.yaw = heading_d
            self.output_pub.publish(output_msg)

            # Publish reference pose
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "odom"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = target_x
            pose_msg.pose.position.y = target_y
            pose_msg.pose.position.z = target_z
            self.ref_pub.publish(pose_msg)

            # Publish errors
            error_msg = Vector3Stamped()
            error_msg.header.frame_id = "odom"
            error_msg.header.stamp = self.get_clock().now().to_msg()
            error_msg.vector.x = e_x
            error_msg.vector.y = e_y
            error_msg.vector.z = e_psi
            self.error_pub.publish(error_msg)

            # Publish desired twist in body frame
            twist_msg = TwistStamped()
            twist_msg.header.frame_id = "base_link"
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            # Transform desired velocities to body frame
            u = surge_velocity * math.cos(e_psi)
            v = surge_velocity * math.sin(e_psi)
            twist_msg.twist.linear.x = u
            twist_msg.twist.linear.y = v
            twist_msg.twist.linear.z = 0.0  # No vertical control as per request
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            self.twist_pub.publish(twist_msg)

            # Publish feedback
            feedback_msg = NavigateWaypoints.Feedback()
            feedback_msg.current_pose = self.current_position
            feedback_msg.current_waypoint_index = self.current_waypoint_index
            goal_handle.publish_feedback(feedback_msg)

            # Logging for debugging
            self.get_logger().info("____________________________________________________")
            self.get_logger().info(f"Current Position: x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f}")
            self.get_logger().info(f"Target Waypoint: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}")
            self.get_logger().info(f"Errors: e_x={e_x:.2f}, e_y={e_y:.2f}, e_psi={e_psi:.2f}")
            self.get_logger().info(f"Desired Velocities: u={u:.2f}, v={v:.2f}")

            # Sleep for a short duration
            time.sleep(0.1)

        goal_handle.succeed()
        result = NavigateWaypoints.Result()
        result.success = success
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = GuidanceActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
