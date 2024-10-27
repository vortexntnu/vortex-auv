#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vortex_msgs.msg import LOSGuidance
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import quat2euler

from guidance_los.los_guidance_computation import LOSGuidanceCalculator


class GuidanceNode(Node):
    """ROS2 node for 3-DOF LOS guidance (surge, pitch, yaw)."""

    def __init__(self):
        super().__init__('guidance_los')

        # Initialize guidance calculator
        self.guidance_calculator = LOSGuidanceCalculator()

        # Publishers
        self.output_pub = self.create_publisher(LOSGuidance, '/guidance/los',
                                                10)
        self.ref_pub = self.create_publisher(PoseStamped,
                                             '/guidance/reference', 10)
        self.error_pub = self.create_publisher(Vector3Stamped,
                                               '/guidance/errors', 10)

        # Subscriber
        self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback,
                                 10)

        # Initialize state variables
        self.current_position = None
        self.current_waypoint_index = 0

        # Example waypoints - modify as needed
        self.waypoints = [
            {
                'x': 5.0,
                'y': 3.0,
                'z': 2.0
            },
            {
                'x': 7.0,
                'y': 5.0,
                'z': 0.0
            },
            {
                'x': 4.0,
                'y': 3.0,
                'z': 1.0
            },
            {
                'x': 4.0,
                'y': 7.0,
                'z': 2.0
            },
        ]

        # Create timer for guidance updates (10 Hz)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('LOS Guidance Node initialized')

    def odom_callback(self, msg: Odometry):
        """Process odometry data to extract position and orientation."""
        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ])

        # Update current position dictionary with all needed states
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'yaw': yaw,
            'pitch': pitch
        }

        # Debug logging
        self.get_logger().debug(f"""
            Position: x={self.current_position['x']:.2f}, 
                     y={self.current_position['y']:.2f}, 
                     z={self.current_position['z']:.2f}
            Orientation: yaw={math.degrees(yaw):.2f}°, 
                        pitch={math.degrees(pitch):.2f}°
        """)

    def timer_callback(self):
        """Execute guidance algorithm periodically."""
        if self.current_position is None:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info(
                'All waypoints reached. Shutting down guidance node.')
            rclpy.shutdown()
            return

        # Get current target waypoint
        target_point = self.waypoints[self.current_waypoint_index]

        # Compute guidance commands
        commands = self.guidance_calculator.compute_los_guidance(
            self.current_position, target_point)

        # Log guidance information
        self.get_logger().info("\n=== Guidance Status ===")
        debug_info = self.guidance_calculator.get_debug_info(commands)
        for key, value in debug_info.items():
            self.get_logger().info(f"{key}: {value}")

        # Check if waypoint is reached
        if commands['distance'] < 0.5:  # 0.5m threshold
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            return

        # Publish all necessary data
        self.publish_guidance(commands)
        self.publish_reference(target_point)
        self.publish_errors(commands)

    def publish_guidance(self, commands):
        """Publish guidance commands."""
        msg = LOSGuidance()
        #msg.header.stamp = self.get_clock().now().to_msg()
        #msg.header.frame_id = "los_guidance"

        # Set guidance commands
        msg.surge = commands['surge']
        msg.pitch = commands['pitch']
        msg.yaw = commands['yaw']

        self.output_pub.publish(msg)

    def publish_reference(self, target):
        """Publish reference pose for visualization."""
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set position
        msg.pose.position.x = target['x']
        msg.pose.position.y = target['y']
        msg.pose.position.z = target['z']

        self.ref_pub.publish(msg)

    def publish_errors(self, commands):
        """Publish errors for monitoring."""
        msg = Vector3Stamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Computing simplified errors for 3-DOF control
        if self.current_position:
            target = self.waypoints[self.current_waypoint_index]
            msg.vector.x = target['x'] - self.current_position[
                'x']  # Position error X
            msg.vector.y = target['y'] - self.current_position[
                'y']  # Position error Y
            msg.vector.z = commands['depth_error']  # Depth error

        self.error_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GuidanceNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
