#!/usr/bin/env python3
"""ROS2 Node for Line-of-Sight (LOS) guidance control using third-order filtering from Fossen.

This node provides waypoint navigation capabilities for underwater vehicles using
LOS guidance algorithm with third-order reference filtering for smooth motion.

The node accepts waypoint navigation goals and generates guidance commands (surge, pitch, yaw)
while providing feedback about the navigation progress.
"""

import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from transforms3d.euler import quat2euler
from vortex_msgs.action import NavigateWaypoints
from vortex_msgs.msg import LOSGuidance, WaypointList

from guidance_los.los_guidance_algorithm import ThirdOrderLOSGuidance


@dataclass(slots=True)
class State:
    """Data class representing the current state of the vehicle.
    Uses slots for memory efficiency.

    Attributes:
        x (float): X position in world frame
        y (float): Y position in world frame
        z (float): Z position (depth) in world frame
        yaw (float): Yaw angle in radians
        pitch (float): Pitch angle in radians
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0

    def as_ndarray(self) -> np.ndarray:
        """Convert state to numpy array format for guidance calculations.

        Returns:
            np.ndarray: Array containing [x, y, z, yaw, pitch]
        """
        return np.array([self.x, self.y, self.z, self.yaw, self.pitch])


class GuidanceActionServer(Node):
    """ROS2 Action Server node for waypoint navigation using LOS guidance.

    This node provides:
    - Action server for waypoint navigation
    - Odometry subscription for vehicle state updates
    - LOS guidance command publishing
    - Reference and error publishing for monitoring
    """

    def __init__(self):
        """Initialize the guidance action server node with all its ROS interfaces."""
        super().__init__('guidance_action_server')

        # Create callback groups for concurrent execution
        self.action_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        # Initialize guidance calculator with third-order filtering
        self.guidance_calculator = ThirdOrderLOSGuidance()

        # Publishers for guidance commands and monitoring
        self.output_pub = self.create_publisher(LOSGuidance, '/guidance/los', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '/guidance/reference', 10)
        self.error_pub = self.create_publisher(Vector3Stamped, '/guidance/errors', 10)

        # Debug publisher for logging
        self.log_publisher = self.create_publisher(
            String, '/guidance_action_server/logs', 10
        )

        # Subscriber for vehicle state updates
        self.create_subscription(
            Odometry,
            '/nucleus/odom',
            self.odom_callback,
            10,
            callback_group=self.timer_cb_group,
        )

        # Publisher for visualizing waypoints in GUI
        self.waypoint_list_pub = self.create_publisher(
            WaypointList, '/guidance/waypoint_list', 10
        )

        # Action server for handling navigation requests
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            execute_callback=self.execute_callback,
            callback_group=self.action_cb_group,
        )

        # State variables initialization
        self.state: State = State()  # Current vehicle state
        self.waypoints = []  # List of waypoints to navigate
        self.current_waypoint_index = 0  # Index of current target waypoint
        self.goal_handle = None  # Handle for current navigation goal
        self.update_period = 0.1  # Guidance update period in seconds

        self.get_logger().info('Guidance Action Server initialized')

    def publish_log(self, message: str):
        """Publish debug log message to ROS topic and node logger.

        Args:
            message (str): Message to be logged
        """
        msg = String()
        msg.data = message
        # self.log_publisher.publish(msg)
        self.get_logger().info(message)

    def publish_waypoint_list(self):
        """Publish the current waypoint list for visualization in GUI.
        Converts waypoints to ROS message format.
        """
        msg = WaypointList()
        msg.waypoints = [Point(x=wp[0], y=wp[1], z=wp[2]) for wp in self.waypoints]
        self.waypoint_list_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        """Process odometry updates and trigger guidance calculations.

        Updates vehicle state and triggers guidance calculations if a navigation
        goal is active.

        Args:
            msg (Odometry): ROS odometry message containing vehicle pose
        """
        try:
            # Extract orientation quaternion to Euler angles
            orientation_q = msg.pose.pose.orientation
            roll, pitch, yaw = quat2euler(
                [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
            )

            # Update vehicle state
            self.state.x = msg.pose.pose.position.x
            self.state.y = msg.pose.pose.position.y
            self.state.z = msg.pose.pose.position.z
            self.state.yaw = yaw
            self.state.pitch = pitch

            # Initialize filter on first callback
            if not hasattr(self, 'filter_initialized'):
                initial_commands = np.array([0.0, 0.0, yaw])
                self.guidance_calculator.reset_filter_state(initial_commands)
                self.filter_initialized = True

            # Log current position for debugging
            self.publish_log(
                f"Position: x={self.state.x:.3f}, "
                f"y={self.state.y:.3f}, "
                f"z={self.state.z:.3f}"
            )
            self.publish_log(
                f"Orientation: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
            )

            # Process guidance if active goal exists
            if self.goal_handle is not None and self.goal_handle.is_active:
                if len(self.waypoints) > 0:
                    self.process_guidance()

        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')

    def process_guidance(self):
        """Process guidance calculations and publish commands.

        Computes guidance commands for current waypoint, publishes commands,
        and handles waypoint transitions.
        """
        try:
            if self.current_waypoint_index >= len(self.waypoints):
                return

            target_point = self.waypoints[self.current_waypoint_index]

            # Compute guidance commands using current state
            commands, distance, depth_error = self.guidance_calculator.compute_guidance(
                self.state.as_ndarray(), target_point, self.update_period
            )

            # Publish commands and monitoring data
            self.publish_guidance(commands)
            self.publish_reference(target_point)
            self.publish_errors(target_point, depth_error)

            # Log guidance status
            self.publish_log(
                f"Guidance Status: surge={commands[0]:.2f}, "
                f"pitch={commands[1]:.2f}, yaw={commands[2]:.2f}"
            )
            self.publish_log(f"Distance to target: {distance:.2f}")

            # Check if waypoint is reached (0.5m threshold)
            if distance < 0.5:
                self.publish_log(f'Reached waypoint {self.current_waypoint_index}')

                # Reset filter state for next waypoint
                initial_commands = np.array([0.0, 0.0, self.state.yaw])
                self.guidance_calculator.reset_filter_state(initial_commands)

                self.current_waypoint_index += 1

                # Check if all waypoints are reached
                if self.current_waypoint_index >= len(self.waypoints):
                    if self.goal_handle and self.goal_handle.is_active:
                        self.goal_handle.succeed()
                        self.goal_handle = None
                    return

            # Publish progress feedback
            if self.goal_handle and self.goal_handle.is_active:
                feedback_msg = NavigateWaypoints.Feedback()
                feedback_msg.current_waypoint_index = float(self.current_waypoint_index)
                self.goal_handle.publish_feedback(feedback_msg)

        except Exception as e:
            self.get_logger().error(f'Error in process_guidance: {str(e)}')

    def execute_callback(self, goal_handle):
        """Execute waypoint navigation action.

        Handles the navigation action request, processes waypoints,
        and monitors completion status.

        Args:
            goal_handle: Action goal handle containing waypoints

        Returns:
            NavigateWaypoints.Result: Success status of navigation
        """
        try:
            self.publish_log('Executing waypoint navigation...')

            # Initialize navigation goal
            self.goal_handle = goal_handle
            self.current_waypoint_index = 0

            # Extract waypoints from goal
            self.waypoints = [
                np.array([wp.pose.position.x, wp.pose.position.y, wp.pose.position.z])
                for wp in goal_handle.request.waypoints
            ]

            # Publish waypoints for visualization
            self.publish_waypoint_list()

            self.publish_log(f'Received {len(self.waypoints)} waypoints')

            # Monitor navigation progress
            while rclpy.ok():
                if not goal_handle.is_active:
                    return NavigateWaypoints.Result(success=False)

                if goal_handle.is_cancel_requested:
                    self.publish_log('Goal canceled')
                    goal_handle.canceled()
                    self.goal_handle = None
                    return NavigateWaypoints.Result(success=False)

                if self.current_waypoint_index >= len(self.waypoints):
                    self.publish_log('All waypoints reached')
                    return NavigateWaypoints.Result(success=True)

                time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f'Error in execute_callback: {str(e)}')
            return NavigateWaypoints.Result(success=False)

    def publish_guidance(self, commands):
        """Publish guidance commands for vehicle control.

        Args:
            commands (np.ndarray): Array of [surge, pitch, yaw] commands
        """
        msg = LOSGuidance()
        msg.surge = commands[0]
        msg.pitch = commands[1]
        msg.yaw = commands[2]
        self.output_pub.publish(msg)

    def publish_reference(self, target):
        """Publish reference pose for monitoring.

        Args:
            target (np.ndarray): Target position [x, y, z]
        """
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        self.ref_pub.publish(msg)

    def publish_errors(self, target, depth_error):
        """Publish position errors for monitoring.

        Args:
            target (np.ndarray): Target position [x, y, z]
            depth_error (float): Error in depth
        """
        msg = Vector3Stamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = target[0] - self.state.x
        msg.vector.y = target[1] - self.state.y
        msg.vector.z = depth_error
        self.error_pub.publish(msg)


def main(args=None):
    """Main function to initialize and run the guidance node.

    Args:
        args: Command line arguments (unused)
    """
    rclpy.init(args=args)

    # Create and initialize node
    action_server = GuidanceActionServer()

    # Create and use multithreaded executor for concurrent processing
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except Exception as e:
        action_server.get_logger().error(f'Error: {str(e)}')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
