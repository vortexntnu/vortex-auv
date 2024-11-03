#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from guidance_los.integrated_los_guidance import IntegratedLOSGuidance
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

        # Initialize integrated LOS guidance calculator
        self.guidance_calculator = IntegratedLOSGuidance()

        # Publishers
        self.output_pub = self.create_publisher(LOSGuidance, '/guidance/los',
                                                10)
        self.ref_pub = self.create_publisher(PoseStamped,
                                             '/guidance/reference', 10)

        # Subscriber to odometry
        self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback,
                                 10)

        # Action server
        self._action_server = ActionServer(self, NavigateWaypoints,
                                           'navigate_waypoints',
                                           self.execute_callback)

        # Initialize state variables
        self.current_position = None  # Will store numpy array [x, y, z, yaw, pitch]
        self.waypoints = []  # Will store list of numpy arrays [x, y, z]
        self.current_waypoint_index = 0
        self.goal_handle = None
        self.update_period = 0.1  # seconds, for filter time step

        self.get_logger().info('Integrated Guidance Action Server has been initialized')

    def odom_callback(self, msg):
        """Process odometry data to extract position and orientation."""
        try:
            # Add debug print
            self.get_logger().info("Received odometry data")

            # Extract orientation
            orientation_q = msg.pose.pose.orientation
            roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, 
                                         orientation_q.y, orientation_q.z])
            
                        # Debug print for position and orientation data
            self.get_logger().info("\n=== Odometry Data ===")
            self.get_logger().info(f"Position: x={msg.pose.pose.position.x:.3f}, "
                                f"y={msg.pose.pose.position.y:.3f}, "
                                f"z={msg.pose.pose.position.z:.3f}")
            self.get_logger().info(f"Orientation (euler): roll={roll:.3f}, "
                                f"pitch={pitch:.3f}, "
                                f"yaw={yaw:.3f}")
            
            # Initialize filter state on first odometry message
            if self.current_position is None:
                initial_commands = np.array([0.0, 0.0, yaw])
                self.guidance_calculator.reset_filter_state(initial_commands)
                self.get_logger().debug('Filter state initialized')

            # Update current position as numpy array [x, y, z, yaw, pitch]
            self.current_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                yaw,
                pitch
            ])

            # Validate current position array
            if not isinstance(self.current_position, np.ndarray) or self.current_position.shape != (5,):
                raise ValueError("Invalid current position format")

            # Process guidance if we have an active goal and waypoints
            if self.goal_handle is not None and self.goal_handle.is_active:
                if len(self.waypoints) > 0:
                    self.process_guidance()

            # Debug the guidance trigger conditions
            self.get_logger().info("Guidance Trigger Check:")
            self.get_logger().info(f"Goal handle exists: {self.goal_handle is not None}")
            self.get_logger().info(f"Goal handle active: {self.goal_handle.is_active if self.goal_handle else False}")
            self.get_logger().info(f"Waypoints available: {len(self.waypoints) > 0}")

            # Process guidance if we have an active goal and waypoints
            if self.goal_handle is not None and self.goal_handle.is_active:
                if len(self.waypoints) > 0:
                    self.get_logger().info("Triggering process_guidance()")
                    self.process_guidance()

        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')

    def process_guidance(self):
        """Process guidance calculations and publish commands."""
        try:
            
            # Debug prints to trace execution flow
            self.get_logger().info("=== Process Guidance ===")
            self.get_logger().info(f"Current position: {self.current_position}")
            self.get_logger().info(f"Current waypoint index: {self.current_waypoint_index}")
            self.get_logger().info(f"Number of waypoints: {len(self.waypoints)}")
            
            if self.current_position is None:
                self.get_logger().warn("No current position available")
                return
                
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().warn("Waypoint index out of range")
                return

            # Get current target waypoint
            target_point = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f"Target waypoint: {target_point}")

            if self.current_position is None or self.current_waypoint_index >= len(self.waypoints):
                return

            # Get current target waypoint (numpy array [x, y, z])
            target_point = self.waypoints[self.current_waypoint_index]

            # Validate waypoint format
            if not isinstance(target_point, np.ndarray) or target_point.shape != (3,):
                raise ValueError("Invalid waypoint format")

            # Validate current position format
            if not isinstance(self.current_position, np.ndarray) or self.current_position.shape != (5,):
                raise ValueError("Invalid current position format")

            # Compute filtered guidance commands
            filtered_commands, distance, depth_error = self.guidance_calculator.compute_guidance(
                self.current_position,
                target_point,
                self.update_period
            )


            self.get_logger().info(f"Computed commands: surge={filtered_commands[0]:.2f}, "
                      f"pitch={filtered_commands[1]:.2f}, "
                      f"yaw={filtered_commands[2]:.2f}")

            # Publish guidance commands and reference
            self.publish_guidance_messages(filtered_commands, target_point)

            # Check if waypoint is reached
            if distance < 0.5:  # waypoint threshold
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
                
                # Reset filter state before moving to next waypoint
                initial_commands = np.array([0.0, 0.0, self.current_position[3]])
                self.guidance_calculator.reset_filter_state(initial_commands)
                
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
            self.log_status(filtered_commands, target_point, distance, depth_error)

        except Exception as e:
            self.get_logger().error(f'Error in process_guidance: {str(e)}')

    def execute_callback(self, goal_handle):
        """Execute the waypoint navigation action."""
        try:
            self.get_logger().info('Executing waypoint navigation...')
            self.get_logger().info(f'Number of waypoints received: {len(goal_handle.request.waypoints)}')

            # Store goal handle and reset waypoint index
            self.goal_handle = goal_handle
            self.current_waypoint_index = 0

            # Extract waypoints as numpy arrays
            self.waypoints = [
                np.array([
                    wp.pose.position.x,
                    wp.pose.position.y,
                    wp.pose.position.z
                ]) for wp in goal_handle.request.waypoints
            ]

            # Validate waypoints
            for i, wp in enumerate(self.waypoints):
                if not isinstance(wp, np.ndarray) or wp.shape != (3,):
                    raise ValueError(f"Invalid waypoint format at index {i}")

            # Initialize filter state with current yaw
            if self.current_position is not None:
                initial_commands = np.array([0.0, 0.0, self.current_position[3]])
                self.guidance_calculator.reset_filter_state(initial_commands)

            # Wait for completion or cancellation
            while rclpy.ok():
                if not goal_handle.is_active:
                    self.goal_handle = None
                    return NavigateWaypoints.Result(success=False)
                
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal canceled')
                    # Reset filter state on cancellation
                    if self.current_position is not None:
                        initial_commands = np.array([0.0, 0.0, self.current_position[3]])
                        self.guidance_calculator.reset_filter_state(initial_commands)
                    goal_handle.canceled()
                    self.goal_handle = None
                    return NavigateWaypoints.Result(success=False)

                # Check if all waypoints are reached
                if self.current_waypoint_index >= len(self.waypoints):
                    self.get_logger().info('All waypoints reached')
                    self.goal_handle = None
                    return NavigateWaypoints.Result(success=True)

                time.sleep(0.1)  # Small sleep to prevent CPU overload

            return NavigateWaypoints.Result(success=False)

        except Exception as e:
            self.get_logger().error(f'Error in execute_callback: {str(e)}')
            return NavigateWaypoints.Result(success=False)

    def publish_guidance_messages(self, commands: np.ndarray, target_point: np.ndarray):
        """Publish guidance commands and reference pose."""
        try:
            # Guidance commands
            los_msg = LOSGuidance()
            los_msg.header.stamp = self.get_clock().now().to_msg()
            los_msg.header.frame_id = "world_ned"
            los_msg.surge = commands[0]  # surge
            los_msg.pitch = commands[1]  # pitch
            los_msg.yaw = commands[2]    # yaw
            self.output_pub.publish(los_msg)

            # Reference pose
            ref_msg = PoseStamped()
            ref_msg.header.frame_id = "world_ned"
            ref_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Set position
            ref_msg.pose.position.x = target_point[0]
            ref_msg.pose.position.y = target_point[1]
            ref_msg.pose.position.z = target_point[2]
            
            # Set orientation (identity quaternion)
            ref_msg.pose.orientation.w = 1.0
            ref_msg.pose.orientation.x = 0.0
            ref_msg.pose.orientation.y = 0.0
            ref_msg.pose.orientation.z = 0.0
            
            self.ref_pub.publish(ref_msg)

        except Exception as e:
            self.get_logger().error(f'Error in publish_guidance_messages: {str(e)}')

    def log_status(self, commands: np.ndarray, target_point: np.ndarray, 
                  distance: float, depth_error: float):
        """Log current status information."""
        try:
            self.get_logger().info("\n=== Guidance Status ===")
            self.get_logger().info(f"Current Position: x={self.current_position[0]:.2f}, "
                                 f"y={self.current_position[1]:.2f}, "
                                 f"z={self.current_position[2]:.2f}")
            self.get_logger().info(f"Target Position: x={target_point[0]:.2f}, "
                                 f"y={target_point[1]:.2f}, "
                                 f"z={target_point[2]:.2f}")
            self.get_logger().info(f"Filtered Commands: surge={commands[0]:.2f} m/s, "
                                 f"pitch={commands[1]:.2f} rad, "
                                 f"yaw={commands[2]:.2f} rad")
            self.get_logger().info(f"Distance to target: {distance:.2f} m")
            self.get_logger().info(f"Depth error: {depth_error:.2f} m")

        except Exception as e:
            self.get_logger().error(f'Error in log_status: {str(e)}')


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
