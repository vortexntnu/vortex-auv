#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from guidance_los.los_guidance_computation import LOSGuidanceCalculator
from vortex_msgs.action import NavigateWaypoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vortex_msgs.msg import LOSGuidance
from transforms3d.euler import quat2euler
import numpy as np
import time

class GuidanceActionServer(Node):
    def __init__(self):
        super().__init__('guidance_action_server')

        # Initialize LOS Calculator
        self.los_calculator = LOSGuidanceCalculator()

        # Publishers
        self.output_pub = self.create_publisher(LOSGuidance, '/guidance/los', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '/guidance/reference', 10)

        # Subscriber to odometry
        self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            self.execute_callback
        )

        # Initialize state variables
        self.current_position = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.goal_handle = None

        self.get_logger().info('Guidance Action Server has been initialized')

    def odom_callback(self, msg):
        """Process odometry data to extract position and orientation."""
        orientation_q = msg.pose.pose.orientation
        _, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
        
        self.current_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'yaw': yaw,
            'pitch': pitch
        }

        # Process guidance if we have waypoints
        if self.goal_handle is not None and self.goal_handle.is_active:
            self.process_guidance()

    def process_guidance(self):
        """Process guidance calculations and publish commands."""
        if not self.current_position or self.current_waypoint_index >= len(self.waypoints):
            return

        # Get current waypoint
        target_point = self.waypoints[self.current_waypoint_index]

        # Compute guidance commands
        commands = self.los_calculator.compute_los_guidance(
            self.current_position,
            target_point
        )

        # Publish guidance commands
        self.publish_guidance_messages(commands, target_point)

        # Check if waypoint is reached
        if commands['distance'] < 0.5:  # waypoint threshold
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1

            # If all waypoints are reached, complete the action
            if self.current_waypoint_index >= len(self.waypoints):
                if self.goal_handle and self.goal_handle.is_active:
                    self.goal_handle.succeed()
                    self.goal_handle = None
                return

        # Publish feedback
        if self.goal_handle and self.goal_handle.is_active:
            feedback_msg = NavigateWaypoints.Feedback()
            feedback_msg.current_waypoint_index = float(self.current_waypoint_index)
            self.goal_handle.publish_feedback(feedback_msg)

        # Log status
        self.log_status(commands, target_point)

    def execute_callback(self, goal_handle):
        """Execute the waypoint navigation action."""
        self.get_logger().info('Executing waypoint navigation...')

        # Store goal handle and reset waypoint index
        self.goal_handle = goal_handle
        self.current_waypoint_index = 0

        # Extract waypoints
        self.waypoints = [{
            'x': wp.pose.position.x,
            'y': wp.pose.position.y,
            'z': wp.pose.position.z
        } for wp in goal_handle.request.waypoints]

        # Wait for completion or cancellation
        while rclpy.ok():
            if not goal_handle.is_active:
                self.goal_handle = None
                return NavigateWaypoints.Result(success=False)
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                self.goal_handle = None
                return NavigateWaypoints.Result(success=False)

            # Check if all waypoints are reached
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints reached')
                self.goal_handle = None
                return NavigateWaypoints.Result(success=True)

            # Small sleep to prevent CPU overload
            time.sleep(0.1)

        return NavigateWaypoints.Result(success=False)

    def publish_guidance_messages(self, commands, target_point):
        """Publish guidance commands and reference pose."""
        # Guidance commands
        los_msg = LOSGuidance()
        los_msg.surge = commands['surge']
        los_msg.pitch = commands['pitch']
        los_msg.yaw = commands['yaw']
        self.output_pub.publish(los_msg)

        # Reference pose
        ref_msg = PoseStamped()
        ref_msg.header.frame_id = "odom"
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.pose.position.x = target_point['x']
        ref_msg.pose.position.y = target_point['y']
        ref_msg.pose.position.z = target_point['z']
        self.ref_pub.publish(ref_msg)

    def log_status(self, commands, target_point):
        """Log current status information."""
        self.get_logger().info("\n=== Guidance Status ===")
        self.get_logger().info(f"Current Position: x={self.current_position['x']:.2f}, "
                              f"y={self.current_position['y']:.2f}, "
                              f"z={self.current_position['z']:.2f}")
        self.get_logger().info(f"Target Position: x={target_point['x']:.2f}, "
                              f"y={target_point['y']:.2f}, "
                              f"z={target_point['z']:.2f}")
        self.get_logger().info(f"Commands: surge={commands['surge']:.2f} m/s, "
                              f"pitch={commands['pitch']:.2f} rad, "
                              f"yaw={commands['yaw']:.2f} rad")
        self.get_logger().info(f"Distance to target: {commands['distance']:.2f} m")
        self.get_logger().info(f"Depth error: {commands['depth_error']:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    action_server = GuidanceActionServer()
    
    try:
        rclpy.spin(action_server)
    except Exception as e:
        action_server.get_logger().error(f'An error occurred: {str(e)}')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()