#!/usr/bin/env python3

import os
from dataclasses import dataclass

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from guidance_los.los_guidance_algorithm import ThirdOrderLOSGuidance
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from transforms3d.euler import quat2euler
from vortex_msgs.action import NavigateWaypoints
from vortex_msgs.msg import LOSGuidance


@dataclass(slots=True)
class State:
    """Data class representing the current state of the vehicle."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0

    def as_ndarray(self) -> np.ndarray:
        """Convert state to numpy array for guidance calculations."""
        return np.array([self.x, self.y, self.z, self.yaw, self.pitch])


class GuidanceActionServer(Node):
    """ROS2 Action Server node for waypoint navigation using LOS guidance."""

    def __init__(self):
        super().__init__('guidance_action_server')

        # Get the configuration file path from parameters
        config_path = self.declare_parameter('config_path', '').value

        # Load configuration
        if config_path and os.path.exists(config_path):
            self.get_logger().info(f"Loading parameters from: {config_path}")
            try:
                with open(config_path) as file:
                    config_params = yaml.safe_load(file)
                    node_params = config_params.get('los_guidance_node', {}).get(
                        'ros__parameters', {}
                    )

                    # Extract different parameter groups
                    self.los_params = node_params.get('los_guidance', {})
                    self.topic_params = node_params.get('topics', {})
                    self.qos_params = node_params.get('qos', {})

                    self.get_logger().info("Configuration loaded successfully")
            except Exception as e:
                self.get_logger().error(f"Error loading config file: {e}")
                self.los_params = {}
                self.topic_params = {}
                self.qos_params = {}
        else:
            self.get_logger().warn("No config file path provided. Using defaults.")
            self.los_params = {}
            self.topic_params = {}
            self.qos_params = {}

        # Initialize filter status flag
        self.filter_initialized = False

        # Add debug parameter
        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').value

        # Create callback groups for concurrent execution
        self.action_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        # Initialize parameters and topics
        self._declare_topic_parameters()
        self._initialize_publishers()
        self._initialize_subscribers()

        # Initialize guidance calculator with third-order filtering
        self.guidance_calculator = ThirdOrderLOSGuidance(self.los_params)

    def _declare_topic_parameters(self):
        """Declare parameters for all topics."""
        # Publishers
        self.declare_parameter(
            'topics.publishers.los_commands', '/guidance/LOS_commands'
        )
        self.declare_parameter(
            'topics.publishers.debug.reference', '/guidance/debug/reference'
        )
        self.declare_parameter(
            'topics.publishers.debug.errors', '/guidance/debug/errors'
        )
        self.declare_parameter('topics.publishers.debug.logs', '/guidance/debug/logs')

        # Subscribers
        self.declare_parameter('topics.subscribers.odometry', '/nucleus/odom')

        # QoS settings
        self.declare_parameter('qos.publisher_depth', 10)
        self.declare_parameter('qos.subscriber_depth', 10)

    def _initialize_publishers(self):
        """Initialize all publishers."""
        pub_qos_depth = self.get_parameter('qos.publisher_depth').value

        # Main guidance command publisher
        los_commands_topic = self.get_parameter('topics.publishers.los_commands').value
        self.guidance_cmd_pub = self.create_publisher(
            LOSGuidance, los_commands_topic, pub_qos_depth
        )

        # Debug publishers
        if self.debug_mode:
            reference_topic = self.get_parameter(
                'topics.publishers.debug.reference'
            ).value
            errors_topic = self.get_parameter('topics.publishers.debug.errors').value
            logs_topic = self.get_parameter('topics.publishers.debug.logs').value

            self.guidance_ref_pub = self.create_publisher(
                PoseStamped, reference_topic, pub_qos_depth
            )
            self.guidance_error_pub = self.create_publisher(
                Vector3Stamped, errors_topic, pub_qos_depth
            )
            self.log_publisher = self.create_publisher(
                String, logs_topic, pub_qos_depth
            )

    def _initialize_subscribers(self):
        """Initialize subscribers."""
        sub_qos_depth = self.get_parameter('qos.subscriber_depth').value
        odom_topic = self.get_parameter('topics.subscribers.odometry').value

        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            sub_qos_depth,
            callback_group=self.timer_cb_group,
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
        self.state: State = State()
        self.waypoints: list[np.ndarray] = []
        self.current_waypoint_index: int = 0
        self.goal_handle: ServerGoalHandle[NavigateWaypoints] | None = None
        self.update_period = 0.1

        self.get_logger().info('Guidance Action Server initialized')

    def publish_log(self, message: str):
        """Publish debug log message to ROS topic and node logger (debug mode only)."""
        if not self.debug_mode:
            return

        msg = String()
        msg.data = message
        self.get_logger().info(message)

    def odom_callback(self, msg: Odometry):
        """Process odometry updates and trigger guidance calculations."""
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
            if not self.filter_initialized:
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

        except Exception as e:
            self.get_logger().error(f'Error in odom_callback: {str(e)}')

    def process_guidance(self):
        """Process guidance calculations and publish control commands."""
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

    def execute_callback(self, goal_handle: ServerGoalHandle[NavigateWaypoints]):
        """Execute waypoint navigation action."""
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

            self.publish_log(f'Received {len(self.waypoints)} waypoints')

            # Monitor navigation progress
            rate = self.create_rate(10)
            while rclpy.ok():
                if not goal_handle.is_active:
                    return NavigateWaypoints.Result(success=False)

                if goal_handle.is_cancel_requested:
                    self.publish_log('Goal canceled')
                    goal_handle.canceled()
                    self.goal_handle = None
                    return NavigateWaypoints.Result(success=False)

                # process guiance
                if len(self.waypoints) > 0:
                    self.process_guidance()

                if self.current_waypoint_index >= len(self.waypoints):
                    self.publish_log('All waypoints reached')
                    return NavigateWaypoints.Result(success=True)

                rate.sleep()

        except Exception as e:
            self.get_logger().error(f'Error in execute_callback: {str(e)}')
            return NavigateWaypoints.Result(success=False)

    def publish_guidance(self, commands):
        """Publish guidance commands for vehicle control."""
        msg = LOSGuidance()
        msg.surge = commands[0]
        msg.pitch = commands[1]
        msg.yaw = commands[2]
        self.guidance_cmd_pub.publish(msg)

    def publish_reference(self, target):
        """Publish reference pose for monitoring (if debug mode is enabled)."""
        if not self.debug_mode:
            return

        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        self.guidance_ref_pub.publish(msg)

    def publish_errors(self, target, depth_error):
        """Publish position errors for monitoring (if debug mode is enabled)."""
        if not self.debug_mode:
            return

        msg = Vector3Stamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = target[0] - self.state.x
        msg.vector.y = target[1] - self.state.y
        msg.vector.z = depth_error
        self.guidance_error_pub.publish(msg)


def main(args=None):
    """Main function to initialize and run the guidance node."""
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
