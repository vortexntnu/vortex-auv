# guidance_los.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vortex_msgs.msg import LOSGuidance
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from transforms3d.euler import quat2euler
from guidance_los.integrated_los_guidance import IntegratedLOSGuidance

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_los')
        
        # Initialize integrated guidance calculator
        self.guidance_calculator = IntegratedLOSGuidance()

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
        self.current_position = None  # Will store numpy array [x, y, z, yaw, pitch]
        self.current_waypoint_index = 0
        self.last_update_time = self.get_clock().now()
        
        # Example waypoints as numpy arrays [x, y, z]
        self.waypoints = [
            np.array([5.0, 3.0, 2.0]),
            np.array([7.0, 5.0, 0.0]),
            np.array([4.0, 3.0, 1.0]),
            np.array([4.0, 7.0, 2.0]),
        ]

        # Create timer for guidance updates (10 Hz)
        self.update_period = 0.1  # seconds
        self.create_timer(self.update_period, self.timer_callback)
        
        self.get_logger().info('Integrated LOS Guidance Node initialized')

    def odom_callback(self, msg: Odometry):
        """Process odometry data to extract position and orientation."""


        # Extract orientation
        orientation_q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, 
                                     orientation_q.y, orientation_q.z])
        
        # First time receiving odometry
        if self.current_position is None:
            initial_commands = np.array([0.0, 0.0, yaw])  # surge, pitch, current yaw
            self.guidance_calculator.reset_filter_state(initial_commands)
        
        # Update current position as numpy array [x, y, z, yaw, pitch]
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            yaw,
            pitch
        ])

        # Debug logging
        self.get_logger().debug(f"""
            Position: x={self.current_position[0]:.2f}, 
                     y={self.current_position[1]:.2f}, 
                     z={self.current_position[2]:.2f}
            Orientation: yaw={math.degrees(self.current_position[3]):.2f}°, 
                        pitch={math.degrees(self.current_position[4]):.2f}°
        """)

    def timer_callback(self):
        if self.current_position is None:
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info(
                'All waypoints reached. Shutting down guidance node.')
            rclpy.shutdown()
            return

        # Get current target waypoint (already numpy array)
        target_point = self.waypoints[self.current_waypoint_index]

        # Compute filtered guidance commands
        # Returns: filtered_commands [surge, pitch, yaw], distance, depth_error
        commands, distance, depth_error = self.guidance_calculator.compute_guidance(
            self.current_position,
            target_point,
            self.update_period
        )

        # Log guidance information
        self.get_logger().info("\n=== Guidance Status ===")
        self.get_logger().info(f"Surge: {commands[0]:.2f}")
        self.get_logger().info(f"Pitch: {commands[1]:.2f}")
        self.get_logger().info(f"Yaw: {commands[2]:.2f}")
        self.get_logger().info(f"Distance: {distance:.2f}")
        self.get_logger().info(f"Depth Error: {depth_error:.2f}")

        # Check if waypoint is reached
        if distance < 0.5:  # 0.5m threshold
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')

            # Reset filter state before moving to next waypoint
            initial_commands = np.array([0.0, 0.0, self.current_position[3]])  # surge, pitch, current yaw
            self.guidance_calculator.reset_filter_state(initial_commands)

            self.current_waypoint_index += 1
            return

        # Publish all necessary data
        self.publish_guidance(commands)
        self.publish_reference(target_point)
        self.publish_errors(target_point, depth_error)


    def publish_guidance(self, commands: np.ndarray):
        """Publish guidance commands."""
        msg = LOSGuidance()
        
        # Set guidance commands from numpy array [surge, pitch, yaw]
        msg.surge = commands[0]
        msg.pitch = commands[1]
        msg.yaw = commands[2]
        
        self.output_pub.publish(msg)

    def publish_reference(self, target: np.ndarray):
        """Publish reference pose for visualization."""
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position from numpy array [x, y, z]
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        
        self.ref_pub.publish(msg)

    def publish_errors(self, target: np.ndarray, depth_error: float):
        """Publish errors for monitoring."""
        msg = Vector3Stamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Computing simplified errors for 3-DOF control
        if self.current_position is not None:
            msg.vector.x = target[0] - self.current_position[0]  # Position error X
            msg.vector.y = target[1] - self.current_position[1]  # Position error Y
            msg.vector.z = depth_error                          # Depth error
            
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
