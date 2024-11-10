# los_guidance_action_server.py
# location: vortex-auv/guidance/guidance_los/guidance_los/los_guidance_action_server.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from vortex_msgs.action import NavigateWaypoints
from vortex_msgs.msg import LOSGuidance
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import numpy as np
import time
from std_msgs.msg import String
from guidance_los.los_guidance_algorithm import FirstOrderLOSGuidance
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class GuidanceActionServer(Node):
    def __init__(self):
        super().__init__('guidance_action_server')

        # Initialize guidance calculator
        self.guidance_calculator = FirstOrderLOSGuidance()

        # Publishers
        self.output_pub = self.create_publisher(LOSGuidance, '/guidance/los', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '/guidance/reference', 10)
        self.error_pub = self.create_publisher(Vector3Stamped, '/guidance/errors', 10)
        
        # Debug publisher
        self.log_publisher = self.create_publisher(String, '/guidance_action_server/logs', 10)

        # Subscriber
        self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)
        
        # Add this line - waypoint list publisher
        self.waypoint_list_pub = self.create_publisher(Float32MultiArray, '/guidance/waypoint_list', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            self.execute_callback
        )

        # State variables
        self.current_position = None  # [x, y, z, yaw, pitch]
        self.waypoints = []
        self.current_waypoint_index = 0
        self.goal_handle = None
        self.update_period = 0.1

        self.get_logger().info('Guidance Action Server initialized')
        
        # Move these methods inside the class with proper indentation
    def publish_log(self, message: str):
        """Publish debug log message."""
        msg = String()
        msg.data = message
        self.log_publisher.publish(msg)
        self.get_logger().info(message)

    def publish_waypoint_list(self):
        """Publish waypoint list for GUI visualization."""
        msg = Float32MultiArray()
        waypoint_data = []
        for wp in self.waypoints:
            waypoint_data.extend(wp.tolist())
        msg.data = waypoint_data
        
        msg.layout.dim = [
            MultiArrayDimension(label="waypoints", size=len(self.waypoints)),
            MultiArrayDimension(label="coordinates", size=3)
        ]
        
        self.waypoint_list_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        """Process odometry data and update guidance if active."""
        try:
            # Extract orientation
            orientation_q = msg.pose.pose.orientation
            roll, pitch, yaw = quat2euler([orientation_q.w, orientation_q.x, 
                                         orientation_q.y, orientation_q.z])
            
            # Initialize filter on first message
            if self.current_position is None:
                initial_commands = np.array([0.0, 0.0, yaw])
                self.guidance_calculator.reset_filter_state(initial_commands)

            # Update current position
            self.current_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                yaw,
                pitch
            ])

            # Log current position
            self.publish_log(
                f"Position: x={self.current_position[0]:.3f}, "
                f"y={self.current_position[1]:.3f}, "
                f"z={self.current_position[2]:.3f}"
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
        """Process guidance calculations and publish commands."""
        
        self.publish_log(f"Processing guidance for target: {target_point}")
        self.publish_log(f"Current position: {self.current_position}")
        try:
            if self.current_waypoint_index >= len(self.waypoints):
                return

            target_point = self.waypoints[self.current_waypoint_index]
            
            
            # Compute guidance commands
            commands, distance, depth_error = self.guidance_calculator.compute_guidance(
                self.current_position,
                target_point,
                self.update_period
            )

            # Publish commands and status
            self.publish_guidance(commands)
            self.publish_reference(target_point)
            self.publish_errors(target_point, depth_error)

            # Log guidance status
            self.publish_log(
                f"Guidance Status: surge={commands[0]:.2f}, "
                f"pitch={commands[1]:.2f}, yaw={commands[2]:.2f}"
            )
            self.publish_log(f"Distance to target: {distance:.2f}")

            # Check if waypoint is reached
            if distance < 0.5:  # 0.5m threshold
                self.publish_log(f'Reached waypoint {self.current_waypoint_index}')
                
                # Reset filter state
                initial_commands = np.array([0.0, 0.0, self.current_position[3]])
                self.guidance_calculator.reset_filter_state(initial_commands)
                
                self.current_waypoint_index += 1

                # If all waypoints reached, succeed the goal
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

        except Exception as e:
            self.get_logger().error(f'Error in process_guidance: {str(e)}')

    def execute_callback(self, goal_handle):
        """Execute the waypoint navigation action."""
        try:
            self.publish_log('Executing waypoint navigation...')
            
            # Store goal handle and reset index
            self.goal_handle = goal_handle
            self.current_waypoint_index = 0

            # Extract waypoints
            self.waypoints = [
                np.array([
                    wp.pose.position.x,
                    wp.pose.position.y,
                    wp.pose.position.z
                ]) for wp in goal_handle.request.waypoints
            ]
            
            # Add this line to publish waypoints for GUI
            self.publish_waypoint_list()
            
            self.publish_log(f'Received {len(self.waypoints)} waypoints')

            # Wait for completion or cancellation
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
        """Publish guidance commands."""
        msg = LOSGuidance()
        msg.surge = commands[0]
        msg.pitch = commands[1]
        msg.yaw = commands[2]
        self.output_pub.publish(msg)

    def publish_reference(self, target):
        """Publish reference pose."""
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        self.ref_pub.publish(msg)

    def publish_errors(self, target, depth_error):
        """Publish position errors."""
        msg = Vector3Stamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = target[0] - self.current_position[0]
        msg.vector.y = target[1] - self.current_position[1]
        msg.vector.z = depth_error
        self.error_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    action_server = GuidanceActionServer()
    
    try:
        rclpy.spin(action_server)
    except Exception as e:
        action_server.get_logger().error(f'Error: {str(e)}')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()