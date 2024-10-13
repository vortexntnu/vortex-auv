# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from nav_msgs.msg import Odometry
# import numpy as np
# import math
# from transforms3d.euler import quat2euler

# class GuidanceNode(Node):
#     def __init__(self):
#         super().__init__('guidance_pkg')

#         # Publisher to output the surge velocity, pitch angle, and heading angle
#         self.output_pub = self.create_publisher(Float32MultiArray, '/guidance/los', 10)

#         # Subscriber to receive odometry data from '/nucleus/odom'
#         self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

#         # Initialize variables
#         self.current_position = None
#         self.current_velocity = None
#         self.current_orientation = None

#         # Parameters for the guidance algorithm
#         self.U = 1.0    # Desired speed
#         self.delta = 1.2  # Lookahead distance

#         # Define waypoints as a list of dictionaries with x, y, z coordinates
#         self.waypoints = [
#             {'x': 5.0, 'y': -5.0, 'z': -8.0},
#             {'x': 10.0, 'y': 5.0, 'z': -5.0},
#             {'x': 15.0, 'y': 0.0, 'z': 0.0},
#             # Add more waypoints as needed
#         ]
#         self.current_waypoint_index = 0

#         # Timer to periodically run the guidance algorithm
#         self.create_timer(0.1, self.timer_callback)

#     def odom_callback(self, msg):
#         """
#         Callback function to process odometry data.
#         """
#         self.current_position = msg.pose.pose.position
#         self.current_velocity = msg.twist.twist.linear
#         orientation_q = msg.pose.pose.orientation

#         # Convert quaternion to Euler angles
#         # ROS quaternion format: (x, y, z, w)
#         quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
#         roll, pitch, yaw = quat2euler(quaternion)
#         self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

#     def timer_callback(self):
#         """
#         Timer callback to compute and publish guidance commands.
#         """
#         if self.current_position is None or self.current_orientation is None:
#             # Wait until odometry data is received
#             return

#         if self.current_waypoint_index >= len(self.waypoints):
#             # All waypoints have been reached
#             self.get_logger().info('All waypoints reached.')
#             return

#         # Get the current waypoint
#         waypoint = self.waypoints[self.current_waypoint_index]

#         # Extract current position
#         current_x = self.current_position.x
#         current_y = self.current_position.y
#         current_z = self.current_position.z

#         # Extract current orientation
#         psi = self.current_orientation['yaw']    # Heading angle
#         theta = self.current_orientation['pitch']  # Pitch angle

#         # Extract target waypoint position
#         target_x = waypoint['x']
#         target_y = waypoint['y']
#         target_z = waypoint['z']

#         # Compute distance to the waypoint
#         distance = math.sqrt(
#             (target_x - current_x) ** 2 +
#             (target_y - current_y) ** 2 +
#             (target_z - current_z) ** 2
#         )

#         # Check if the waypoint is reached
#         if distance < 0.5:
#             self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
#             self.current_waypoint_index += 1
#             if self.current_waypoint_index >= len(self.waypoints):
#                 self.get_logger().info('All waypoints reached.')
#                 return
#             else:
#                 waypoint = self.waypoints[self.current_waypoint_index]

#         # Guidance calculations using a Line-of-Sight (LOS) algorithm
#         ref_x = current_x
#         ref_y = current_y
#         ref_z = current_z

#         dx = target_x - ref_x
#         dy = target_y - ref_y
#         dz = target_z - ref_z

#         # Compute desired heading angle (psi_d)
#         pi_p = math.atan2(dy, dx)
#         pi_p = normalize_angle(pi_p)

#         # Compute desired pitch angle (theta_d)
#         gamma = math.atan2(-dz, math.hypot(dx, dy))

#         # Compute cross-track error
#         e_ct = -(dx * (current_y - ref_y) - dy * (current_x - ref_x)) / math.hypot(dx, dy)

#         # Compute desired course angle
#         course_d = pi_p - math.atan(e_ct / self.delta)
#         course_d = normalize_angle(course_d)

#         # Set desired surge velocity
#         surge_velocity = self.U

#         # Set desired heading and pitch angles
#         heading_angle = course_d
#         pitch_angle = gamma

#         # Prepare and publish the output message
#         output_msg = Float32MultiArray()
#         output_msg.data = [surge_velocity, pitch_angle, heading_angle]
#         self.output_pub.publish(output_msg)

# def normalize_angle(angle):
#     """
#     Normalize an angle to the range [-π, π].
#     """
#     return (angle + np.pi) % (2 * np.pi) - np.pi

# def main(args=None):
#     rclpy.init(args=args)
#     node = GuidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#____________________________________________________________________________________________________________________________
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
# from nav_msgs.msg import Odometry
# import numpy as np
# import math
# from transforms3d.euler import quat2euler

# class GuidanceNode(Node):
#     def __init__(self):
#         super().__init__('guidance_pkg')

#         # Publishers
#         self.output_pub = self.create_publisher(Float32MultiArray, '/guidance/los', 10)
#         self.ref_pub = self.create_publisher(PoseStamped, '/guidance/reference', 10)
#         self.error_pub = self.create_publisher(Vector3Stamped, '/guidance/errors', 10)
#         self.twist_pub = self.create_publisher(TwistStamped, '/guidance/twist', 10)

#         # Subscriber to odometry
#         self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

#         # Initialize variables
#         self.current_position = None
#         self.current_velocity = None
#         self.current_orientation = None
#         self.ref_point = None

#         # Parameters for guidance algorithm
#         self.U = 1.0    # Desired speed
#         self.delta = 1.2  # Lookahead distance

#         # Define waypoints as a list of dictionaries with x, y, z coordinates
#         self.waypoints = [
#             {'x': 5.0, 'y': -5.0, 'z': -8.0},
#             {'x': 10.0, 'y': 5.0, 'z': -5.0},
#             {'x': 15.0, 'y': 0.0, 'z': 0.0},
#         ]
#         self.current_waypoint_index = 0

#         # Timer to run the guidance algorithm periodically
#         self.create_timer(0.1, self.timer_callback)

#     def odom_callback(self, msg):
#         self.current_position = msg.pose.pose
#         self.current_velocity = msg.twist.twist
#         orientation_q = msg.pose.pose.orientation
#         quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
#         roll, pitch, yaw = quat2euler(quaternion)
#         self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

#     def timer_callback(self):
#         if self.current_position is None or self.current_orientation is None:
#             return

#         if self.current_waypoint_index >= len(self.waypoints):
#             self.get_logger().info('All waypoints reached.')
#             return

#         waypoint = self.waypoints[self.current_waypoint_index]

#         # Extract current position and orientation
#         current_x = self.current_position.position.x
#         current_y = self.current_position.position.y
#         current_z = self.current_position.position.z
#         psi = self.current_orientation['yaw']

#         # Extract target waypoint position
#         target_x = waypoint['x']
#         target_y = waypoint['y']
#         target_z = waypoint['z']

#         # Compute distance to the waypoint
#         distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2 + (target_z - current_z) ** 2)

#         # Check if waypoint is reached
#         if distance < 0.5:
#             self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
#             self.current_waypoint_index += 1
#             if self.current_waypoint_index >= len(self.waypoints):
#                 self.get_logger().info('All waypoints reached.')
#                 return

#         # Guidance calculations using LOS algorithm
#         ref_x, ref_y, ref_z = current_x, current_y, current_z
#         dx, dy, dz = target_x - ref_x, target_y - ref_y, target_z - ref_z

#         pi_p = math.atan2(dy, dx)
#         pi_p = normalize_angle(pi_p)

#         gamma = math.atan2(-dz, math.hypot(dx, dy))

#         # Compute along-track and cross-track errors in local reference frame
#         R = np.array([[math.cos(pi_p), -math.sin(pi_p)], 
#                       [math.sin(pi_p), math.cos(pi_p)]])
#         eps = R.T @ np.array([current_x - ref_x, current_y - ref_y])

#         e_x = eps[0]  # Along-track error
#         e_y = eps[1]  # Cross-track error

#         # Compute desired course angle
#         course_d = pi_p - math.atan(e_y / self.delta)
#         course_d = normalize_angle(course_d)

#         # Heading error computation
#         heading_d = course_d
#         e_psi = normalize_angle(heading_d - psi)

#         # Set desired surge velocity, heading, and pitch angles
#         surge_velocity = self.U
#         heading_angle = heading_d
#         pitch_angle = gamma

#         # Prepare and publish the output message
#         output_msg = Float32MultiArray()
#         output_msg.data = [surge_velocity, pitch_angle, heading_angle]
#         self.output_pub.publish(output_msg)

#         # Publish reference pose
#         pose_msg = PoseStamped()
#         pose_msg.header.frame_id = "odom"
#         pose_msg.header.stamp = self.get_clock().now().to_msg()
#         pose_msg.pose.position.x = target_x
#         pose_msg.pose.position.y = target_y
#         pose_msg.pose.position.z = target_z
#         self.ref_pub.publish(pose_msg)

#         # Publish errors
#         error_msg = Vector3Stamped()
#         error_msg.header.frame_id = "odom"
#         error_msg.header.stamp = self.get_clock().now().to_msg()
#         error_msg.vector.x = e_x
#         error_msg.vector.y = e_y
#         error_msg.vector.z = e_psi
#         self.error_pub.publish(error_msg)

#         # Publish desired twist
#         twist_msg = TwistStamped()
#         twist_msg.header.frame_id = "base_link"
#         twist_msg.header.stamp = self.get_clock().now().to_msg()
#         twist_msg.twist.linear.x = surge_velocity * math.cos(heading_d)
#         twist_msg.twist.linear.y = surge_velocity * math.sin(heading_d)
#         self.twist_pub.publish(twist_msg)

# def normalize_angle(angle):
#     return (angle + np.pi) % (2 * np.pi) - np.pi

# def main(args=None):
#     rclpy.init(args=args)
#     node = GuidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
# from nav_msgs.msg import Odometry
# import numpy as np
# import math

# from transforms3d.euler import quat2euler

# def normalize_angle(angle):
#     """Normalize angle to be within [-pi, pi]."""
#     return (angle + np.pi) % (2 * np.pi) - np.pi

# class GuidanceNode(Node):
#     def __init__(self):
#         super().__init__('guidance_pkg')

#         # Publishers
#         self.output_pub = self.create_publisher(Float32MultiArray, '/guidance/los', 10)
#         self.ref_pub = self.create_publisher(PoseStamped, '/guidance/reference', 10)
#         self.error_pub = self.create_publisher(Vector3Stamped, '/guidance/errors', 10)
#         self.twist_pub = self.create_publisher(TwistStamped, '/guidance/twist', 10)

#         # Subscriber to odometry
#         self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

#         # Initialize variables
#         self.current_position = None
#         self.current_velocity = None
#         self.current_orientation = None

#         # Parameters for guidance algorithm
#         self.U = 1.0    # Desired surge speed (m/s)
#         self.delta = 0.2  # Lookahead distance (m)

#         # Define waypoints
#         self.waypoints = [
#             {'x': 5.0, 'y': 5.0, 'z': 8.0},
#             {'x': 10.0, 'y': 5.0, 'z': -5.0},
#             {'x': 15.0, 'y': 0.0, 'z': 0.0},
#         ]
#         self.current_waypoint_index = 0

#         # Timer to run the guidance algorithm periodically
#         self.create_timer(0.1, self.timer_callback)

#     def odom_callback(self, msg):
#         """Callback to handle odometry updates."""
#         self.current_position = msg.pose.pose
#         self.current_velocity = msg.twist.twist
#         orientation_q = msg.pose.pose.orientation
#         quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
#         roll, pitch, yaw = quat2euler(quaternion)
#         self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

#     def timer_callback(self):
#         """Main guidance algorithm executed periodically."""
#         if self.current_position is None or self.current_orientation is None:
#             return

#         if self.current_waypoint_index >= len(self.waypoints):
#             self.get_logger().info('All waypoints reached. Shutting down guidance node.')
#             rclpy.shutdown()
#             return

#         waypoint = self.waypoints[self.current_waypoint_index]

#         # Extract current position and orientation
#         current_x = self.current_position.position.x
#         current_y = self.current_position.position.y
#         current_z = self.current_position.position.z
#         psi = self.current_orientation['yaw']

#         # Extract target waypoint position
#         target_x = waypoint['x']
#         target_y = waypoint['y']
#         target_z = waypoint['z']

#         # Compute distance to the waypoint
#         distance = math.sqrt((target_x - current_x) ** 2 +
#                              (target_y - current_y) ** 2 +
#                              (target_z - current_z) ** 2)

#         # Check if waypoint is reached
#         if distance < 0.1:
#             self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
#             self.current_waypoint_index += 1
#             if self.current_waypoint_index >= len(self.waypoints):
#                 self.get_logger().info('All waypoints reached. Shutting down guidance node.')
#                 rclpy.shutdown()
#                 return
#             else:
#                 waypoint = self.waypoints[self.current_waypoint_index]
#                 target_x = waypoint['x']
#                 target_y = waypoint['y']
#                 target_z = waypoint['z']

#         # Guidance calculations using LOS algorithm
#         # Reference point is the previous waypoint or current position
#         # Define starting point (e.g., origin)
#         starting_point = {'x': 0.0, 'y': 0.0, 'z': 0.0}

#         # Modify the reference point for the first waypoint
#         if self.current_waypoint_index == 0:
#             ref_x = starting_point['x']
#             ref_y = starting_point['y']
#             ref_z = starting_point['z']
#         else:
#             prev_waypoint = self.waypoints[self.current_waypoint_index - 1]
#             ref_x = prev_waypoint['x']
#             ref_y = prev_waypoint['y']
#             ref_z = prev_waypoint['z']

#         # Differences between target and reference point
#         dx = target_x - ref_x
#         dy = target_y - ref_y
#         dz = target_z - ref_z

#         # Desired path angle
#         pi_p = math.atan2(dy, dx)
#         pi_p = normalize_angle(pi_p)

#         # Compute pitch angle (gamma)
#         gamma = math.atan2(-dz, math.hypot(dx, dy))

#         # Compute along-track and cross-track errors
#         # Transform position error into path coordinate frame
#         R = np.array([[math.cos(pi_p), math.sin(pi_p)],
#                       [-math.sin(pi_p), math.cos(pi_p)]])
#         e_pos = np.array([current_x - ref_x, current_y - ref_y])
#         eps = R @ e_pos

#         e_x = eps[0]  # Along-track error
#         e_y = eps[1]  # Cross-track error

#         # Compute desired course angle
#         course_d = pi_p - math.atan2(e_y, self.delta)
#         course_d = normalize_angle(course_d)

#         # Heading error computation
#         heading_d = course_d
#         e_psi = normalize_angle(heading_d - psi)

#         # Set desired surge velocity (could be adjusted based on e_x)
#         surge_velocity = self.U

#         # Prepare and publish the output message
#         output_msg = Float32MultiArray()
#         output_msg.data = [surge_velocity, gamma, heading_d]
#         self.output_pub.publish(output_msg)

#         # Publish reference pose
#         pose_msg = PoseStamped()
#         pose_msg.header.frame_id = "odom"
#         pose_msg.header.stamp = self.get_clock().now().to_msg()
#         pose_msg.pose.position.x = target_x
#         pose_msg.pose.position.y = target_y
#         pose_msg.pose.position.z = target_z
#         self.ref_pub.publish(pose_msg)

#         # Publish errors
#         error_msg = Vector3Stamped()
#         error_msg.header.frame_id = "odom"
#         error_msg.header.stamp = self.get_clock().now().to_msg()
#         error_msg.vector.x = e_x
#         error_msg.vector.y = e_y
#         error_msg.vector.z = e_psi
#         self.error_pub.publish(error_msg)

#         # Publish desired twist in body frame
#         twist_msg = TwistStamped()
#         twist_msg.header.frame_id = "base_link"
#         twist_msg.header.stamp = self.get_clock().now().to_msg()
#         # Transform desired velocities to body frame
#         u = surge_velocity * math.cos(e_psi)
#         v = surge_velocity * math.sin(e_psi)
#         twist_msg.twist.linear.x = u
#         twist_msg.twist.linear.y = v
#         twist_msg.twist.linear.z = 0.0  # No vertical control as per request
#         twist_msg.twist.angular.x = 0.0
#         twist_msg.twist.angular.y = 0.0
#         twist_msg.twist.angular.z = 0.0
#         self.twist_pub.publish(twist_msg)

#        # Logging for debugging
#         self.get_logger().info("______________________________________________________________________")
#         self.get_logger().info(f"Current Position: x={current_x}, y={current_y}, z={current_z}")
#         self.get_logger().info(f"Target Waypoint: x={target_x}, y={target_y}, z={target_z}")
#         self.get_logger().info(f"Errors: e_x={e_x}, e_y={e_y}, e_psi={e_psi}")
#         self.get_logger().info(f"Desired Velocities: u={u}, v={v}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = GuidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
# from nav_msgs.msg import Odometry
# import numpy as np
# import math
# from transforms3d.euler import quat2euler

# def normalize_angle(angle):
#     """Normalize angle to be within [-pi, pi]."""
#     return (angle + np.pi) % (2 * np.pi) - np.pi

# class GuidanceNode(Node):
#     def __init__(self):
#         super().__init__('guidance_pkg')

#         # Publishers
#         self.output_pub = self.create_publisher(Float32MultiArray, '/guidance/los', 10)
#         self.ref_pub = self.create_publisher(PoseStamped, '/guidance/reference', 10)
#         self.error_pub = self.create_publisher(Vector3Stamped, '/guidance/errors', 10)
#         self.twist_pub = self.create_publisher(TwistStamped, '/guidance/twist', 10)

#         # Subscriber to odometry
#         self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

#         # Initialize variables
#         self.current_position = None
#         self.current_velocity = None
#         self.current_orientation = None

#         # Parameters for guidance algorithm
#         self.U = 1.0    # Desired surge speed (m/s)
#         self.delta = 0.2  # Lookahead distance (m)

#         # Define waypoints
#         self.waypoints = [
#             {'x': 5.0, 'y': 5.0, 'z': 8.0},
#             {'x': 10.0, 'y': 5.0, 'z': -5.0},
#             {'x': 15.0, 'y': 0.0, 'z': 0.0},
#         ]
#         self.current_waypoint_index = 0

#         # Define starting point (fixed reference point)
#         self.starting_point = {'x': 0.0, 'y': 0.0, 'z': 0.0}

#         # Timer to run the guidance algorithm periodically
#         self.create_timer(0.1, self.timer_callback)

#     def odom_callback(self, msg):
#         """Callback to handle odometry updates."""
#         self.current_position = msg.pose.pose
#         self.current_velocity = msg.twist.twist
#         orientation_q = msg.pose.pose.orientation
#         quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
#         roll, pitch, yaw = quat2euler(quaternion)
#         self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

#     def timer_callback(self):
#         """Main guidance algorithm executed periodically."""
#         if self.current_position is None or self.current_orientation is None:
#             return

#         if self.current_waypoint_index >= len(self.waypoints):
#             self.get_logger().info('All waypoints reached. Shutting down guidance node.')
#             rclpy.shutdown()
#             return

#         waypoint = self.waypoints[self.current_waypoint_index]

#         # Extract current position and orientation
#         current_x = self.current_position.position.x
#         current_y = self.current_position.position.y
#         current_z = self.current_position.position.z
#         psi = self.current_orientation['yaw']

#         # Extract target waypoint position
#         target_x = waypoint['x']
#         target_y = waypoint['y']
#         target_z = waypoint['z']

#         # Compute distance to the waypoint
#         distance = math.sqrt((target_x - current_x) ** 2 +
#                              (target_y - current_y) ** 2 +
#                              (target_z - current_z) ** 2)

#         # Check if waypoint is reached
#         if distance < 0.1:
#             self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')
#             self.current_waypoint_index += 1
#             if self.current_waypoint_index >= len(self.waypoints):
#                 self.get_logger().info('All waypoints reached. Shutting down guidance node.')
#                 rclpy.shutdown()
#                 return
#             else:
#                 waypoint = self.waypoints[self.current_waypoint_index]
#                 target_x = waypoint['x']
#                 target_y = waypoint['y']
#                 target_z = waypoint['z']

#         # Guidance calculations using LOS algorithm
#         # Reference point is either the starting point or the previous waypoint
#         if self.current_waypoint_index == 0:
#             ref_x = self.starting_point['x']
#             ref_y = self.starting_point['y']
#             ref_z = self.starting_point['z']
#         else:
#             prev_waypoint = self.waypoints[self.current_waypoint_index - 1]
#             ref_x = prev_waypoint['x']
#             ref_y = prev_waypoint['y']
#             ref_z = prev_waypoint['z']

#         # Differences between target and reference point
#         dx = target_x - ref_x
#         dy = target_y - ref_y
#         dz = target_z - ref_z

#         # Desired path angle
#         pi_p = math.atan2(dy, dx)
#         pi_p = normalize_angle(pi_p)

#         # Compute pitch angle (gamma)
#         gamma = math.atan2(-dz, math.hypot(dx, dy))

#         # Compute along-track and cross-track errors
#         # Transform position error into path coordinate frame
#         R = np.array([[math.cos(pi_p), math.sin(pi_p)],
#                       [-math.sin(pi_p), math.cos(pi_p)]])
#         e_pos = np.array([current_x - ref_x, current_y - ref_y])
#         eps = R @ e_pos

#         e_x = eps[0]  # Along-track error
#         e_y = eps[1]  # Cross-track error

#         # Compute desired course angle
#         course_d = pi_p - math.atan2(e_y, self.delta)
#         course_d = normalize_angle(course_d)

#         # Heading error computation
#         heading_d = course_d
#         e_psi = normalize_angle(heading_d - psi)

#         # Set desired surge velocity (could be adjusted based on e_x)
#         surge_velocity = self.U

#         # Prepare and publish the output message
#         output_msg = Float32MultiArray()
#         output_msg.data = [surge_velocity, gamma, heading_d]
#         self.output_pub.publish(output_msg)

#         # Publish reference pose
#         pose_msg = PoseStamped()
#         pose_msg.header.frame_id = "odom"
#         pose_msg.header.stamp = self.get_clock().now().to_msg()
#         pose_msg.pose.position.x = target_x
#         pose_msg.pose.position.y = target_y
#         pose_msg.pose.position.z = target_z
#         self.ref_pub.publish(pose_msg)

#         # Publish errors
#         error_msg = Vector3Stamped()
#         error_msg.header.frame_id = "odom"
#         error_msg.header.stamp = self.get_clock().now().to_msg()
#         error_msg.vector.x = e_x
#         error_msg.vector.y = e_y
#         error_msg.vector.z = e_psi
#         self.error_pub.publish(error_msg)

#         # Publish desired twist in body frame
#         twist_msg = TwistStamped()
#         twist_msg.header.frame_id = "base_link"
#         twist_msg.header.stamp = self.get_clock().now().to_msg()
#         # Transform desired velocities to body frame
#         u = surge_velocity * math.cos(e_psi)
#         v = surge_velocity * math.sin(e_psi)
#         twist_msg.twist.linear.x = u
#         twist_msg.twist.linear.y = v
#         twist_msg.twist.linear.z = 0.0  # No vertical control as per request
#         twist_msg.twist.angular.x = 0.0
#         twist_msg.twist.angular.y = 0.0
#         twist_msg.twist.angular.z = 0.0
#         self.twist_pub.publish(twist_msg)

#         # Logging for debugging
#         self.get_logger().info("______________________________________________________________________")
#         self.get_logger().info(f"Current Position: x={current_x}, y={current_y}, z={current_z}")
#         self.get_logger().info(f"Target Waypoint: x={target_x}, y={target_y}, z={target_z}")
#         self.get_logger().info(f"Errors: e_x={e_x}, e_y={e_y}, e_psi={e_psi}")
#         self.get_logger().info(f"Desired Velocities: u={u}, v={v}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = GuidanceNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()