#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vortex_msgs.msg import LOSGuidance
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from transforms3d.euler import quat2euler


def normalize_angle(angle):
    """Normalize angle to be within [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class GuidanceNode(Node):

    def __init__(self):
        super().__init__('guidance_los')

        # Publishers - keeping the same interface for compatibility
        self.output_pub = self.create_publisher(LOSGuidance, '/guidance/los',
                                                10)
        self.ref_pub = self.create_publisher(PoseStamped,
                                             '/guidance/reference', 10)
        self.error_pub = self.create_publisher(Vector3Stamped,
                                               '/guidance/errors', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/guidance/twist',
                                               10)

        # Subscriber to odometry
        self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback,
                                 10)

        # Initialize variables
        self.current_position = None
        self.current_velocity = None
        self.current_orientation = None

        # Enhanced guidance parameters
        self.nominal_speed = 1.0  # Base desired surge speed (m/s)
        self.min_speed = 0.3  # Minimum allowed speed
        self.delta_min = 1.0  # Minimum lookahead distance
        self.delta_max = 5.0  # Maximum lookahead distance
        self.delta_factor = 3.0  # Multiplier for adaptive lookahead
        self.delta_vertical = 2.0  # Vertical lookahead distance
        self.waypoint_threshold = 0.5  # Distance threshold for waypoint switching

        # Define waypoints
        self.waypoints = [
            {
                'x': 5.0,
                'y': 0.0,
                'z': 0.0
            },
            {
                'x': 10.0,
                'y': 5.0,
                'z': 0.0
            },
            {
                'x': 15.0,
                'y': 5.0,
                'z': 5.0
            },
        ]
        self.current_waypoint_index = 0
        self.starting_point = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Timer for guidance updates
        self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        """Callback to handle odometry updates."""
        self.current_position = msg.pose.pose
        self.current_velocity = msg.twist.twist
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]
        roll, pitch, yaw = quat2euler(quaternion)
        self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    def compute_adaptive_lookahead(self, cross_track_error):
        """Compute adaptive lookahead distance based on cross-track error."""
        delta = self.delta_factor * abs(cross_track_error)
        return np.clip(delta, self.delta_min, self.delta_max)

    def compute_desired_speed(self, course_error, distance_to_waypoint):
        """Compute desired speed based on course error and distance."""
        # Reduce speed for large course corrections
        speed_factor = math.cos(course_error)
        # Additional speed reduction near waypoints
        waypoint_factor = min(1.0, distance_to_waypoint / self.delta_max)

        desired_speed = self.nominal_speed * speed_factor * waypoint_factor
        return max(self.min_speed, desired_speed)

    def compute_3d_los(self, e_y, e_z, pi_p, gamma, current_depth):
        """Compute 3D LOS guidance commands."""
        # Horizontal guidance
        delta_h = self.compute_adaptive_lookahead(e_y)
        chi_d = pi_p + math.atan2(-e_y, delta_h)

        # Vertical guidance with depth rate limiting
        max_pitch = np.pi / 4  # Maximum pitch angle (45 degrees)
        theta_d = np.clip(gamma + math.atan2(-e_z, self.delta_vertical),
                          -max_pitch, max_pitch)

        return chi_d, theta_d

    def timer_callback(self):
        """Main guidance algorithm executed periodically."""
        if self.current_position is None or self.current_orientation is None:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info(
                'All waypoints reached. Shutting down guidance node.')
            rclpy.shutdown()
            return

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

        # Compute distance to waypoint
        distance = math.sqrt((target_x - current_x)**2 +
                             (target_y - current_y)**2 +
                             (target_z - current_z)**2)

        # Check if waypoint is reached
        if distance < self.waypoint_threshold:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info(
                    'All waypoints reached. Shutting down guidance node.')
                rclpy.shutdown()
                return
            waypoint = self.waypoints[self.current_waypoint_index]
            target_x = waypoint['x']
            target_y = waypoint['y']
            target_z = waypoint['z']

        # Reference point selection
        if self.current_waypoint_index == 0:
            ref_x = self.starting_point['x']
            ref_y = self.starting_point['y']
            ref_z = self.starting_point['z']
        else:
            prev_waypoint = self.waypoints[self.current_waypoint_index - 1]
            ref_x = prev_waypoint['x']
            ref_y = prev_waypoint['y']
            ref_z = prev_waypoint['z']

        # Path vector calculations
        dx = target_x - ref_x
        dy = target_y - ref_y
        dz = target_z - ref_z

        # Desired path angles
        pi_p = math.atan2(dy, dx)
        gamma = math.atan2(-dz, math.hypot(dx, dy))

        # Transform position error into path coordinate frame
        R = np.array([[math.cos(pi_p), math.sin(pi_p)],
                      [-math.sin(pi_p), math.cos(pi_p)]])
        e_pos = np.array([current_x - ref_x, current_y - ref_y])
        eps = R @ e_pos

        e_x = eps[0]  # Along-track error
        e_y = eps[1]  # Cross-track error
        e_z = current_z - (ref_z +
                           (dz / math.hypot(dx, dy)) * e_x)  # Vertical error

        # Compute 3D LOS guidance commands
        heading_d, pitch_d = self.compute_3d_los(e_y, e_z, pi_p, gamma,
                                                 current_z)

        # Normalize angles
        heading_d = normalize_angle(heading_d)
        pitch_d = normalize_angle(pitch_d)

        # Compute heading error for speed adjustment
        e_psi = normalize_angle(heading_d - psi)

        # Compute desired surge speed
        surge_velocity = self.compute_desired_speed(e_psi, distance)

        # Prepare and publish the guidance message
        output_msg = LOSGuidance()
        output_msg.surge = surge_velocity
        output_msg.pitch = pitch_d
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
        error_msg.vector.z = e_z
        self.error_pub.publish(error_msg)

        # Log key information
        self.get_logger().info("------- Guidance Update -------")
        self.get_logger().info(
            f"Position: x={current_x:.2f}, y={current_y:.2f}, z={current_z:.2f}"
        )
        self.get_logger().info(
            f"Target: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}")
        self.get_logger().info(
            f"Commands: surge={surge_velocity:.2f}, pitch={pitch_d:.2f}, yaw={heading_d:.2f}"
        )
        self.get_logger().info(
            f"Errors: cross-track={e_y:.2f}, vertical={e_z:.2f}, heading={e_psi:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
