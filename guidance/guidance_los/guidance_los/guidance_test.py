#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from transforms3d.euler import euler2quat
import numpy as np
import math
from rclpy.qos import QoSProfile

class GuidanceTestNode(Node):
    def __init__(self):
        super().__init__('guidance_test')
        
        # Publisher for odometry data
        self.odom_pub = self.create_publisher(
            Odometry,
            '/nucleus/odom',
            10)

        # Initialize simulation parameters
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        self.current_orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.current_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        
        # Test scenarios
        self.test_scenarios = [
            self.straight_line_test,
            self.circular_path_test,
            self.depth_change_test,
            self.stationary_test
        ]
        self.current_scenario = 0
        self.scenario_time = 0.0
        
        # Create timer for publishing test data
        self.create_timer(0.1, self.timer_callback)  # 10 Hz update rate
        
        self.get_logger().info('Guidance Test Node started')
        
    def straight_line_test(self, t):
        """Simulate straight line motion."""
        self.current_position.x = t * 0.5  # Moving at 0.5 m/s in x direction
        self.current_position.y = 0.0
        self.current_position.z = 0.0
        self.current_orientation['yaw'] = 0.0
        self.current_velocity.x = 0.5
        self.current_velocity.y = 0.0
        self.current_velocity.z = 0.0
        
        return t < 20.0  # Run for 20 seconds
        
    def circular_path_test(self, t):
        """Simulate circular motion."""
        radius = 5.0
        angular_velocity = 0.2  # rad/s
        
        self.current_position.x = radius * math.cos(angular_velocity * t)
        self.current_position.y = radius * math.sin(angular_velocity * t)
        self.current_position.z = 0.0
        self.current_orientation['yaw'] = angular_velocity * t + math.pi/2
        self.current_velocity.x = -radius * angular_velocity * math.sin(angular_velocity * t)
        self.current_velocity.y = radius * angular_velocity * math.cos(angular_velocity * t)
        self.current_velocity.z = 0.0
        
        return t < 31.4  # Run for one complete circle
        
    def depth_change_test(self, t):
        """Simulate depth changes."""
        self.current_position.x = t * 0.3
        self.current_position.y = 0.0
        self.current_position.z = 2.0 * math.sin(0.2 * t)  # Sinusoidal depth change
        self.current_orientation['pitch'] = 0.2 * math.cos(0.2 * t)
        self.current_velocity.x = 0.3
        self.current_velocity.y = 0.0
        self.current_velocity.z = 0.4 * math.cos(0.2 * t)
        
        return t < 30.0  # Run for 30 seconds
        
    def stationary_test(self, t):
        """Simulate stationary vehicle."""
        self.current_position.x = 0.0
        self.current_position.y = 0.0
        self.current_position.z = 0.0
        self.current_orientation['yaw'] = 0.0
        self.current_velocity.x = 0.0
        self.current_velocity.y = 0.0
        self.current_velocity.z = 0.0
        
        return t < 10.0  # Run for 10 seconds
        
    def timer_callback(self):
        """Publish test odometry data."""
        if self.current_scenario >= len(self.test_scenarios):
            self.get_logger().info('All test scenarios completed')
            rclpy.shutdown()
            return
            
        # Run current scenario
        scenario_complete = self.test_scenarios[self.current_scenario](self.scenario_time)
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        odom_msg.pose.pose.position = self.current_position
        
        # Convert Euler angles to quaternion
        quat = euler2quat(
            self.current_orientation['roll'],
            self.current_orientation['pitch'],
            self.current_orientation['yaw']
        )
        odom_msg.pose.pose.orientation = Quaternion(
            w=quat[0], x=quat[1], y=quat[2], z=quat[3]
        )
        
        # Set velocity
        odom_msg.twist.twist.linear = self.current_velocity
        
        # Publish message
        self.odom_pub.publish(odom_msg)
        
        # Update scenario time or switch to next scenario
        if scenario_complete:
            self.scenario_time += 0.1
        else:
            self.current_scenario += 1
            self.scenario_time = 0.0
            scenario_names = ["Straight Line", "Circular Path", "Depth Change", "Stationary"]
            if self.current_scenario < len(scenario_names):
                self.get_logger().info(f'Starting {scenario_names[self.current_scenario]} test')

def main(args=None):
    rclpy.init(args=args)
    node = GuidanceTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# from transforms3d.euler import euler2quat
# import numpy as np
# import math

# class GuidanceTestNode(Node):
#     def __init__(self):
#         super().__init__('guidance_test')
        
#         # Publisher for odometry data
#         self.odom_pub = self.create_publisher(
#             Odometry,
#             '/nucleus/odom',
#             10)

#         # Initialize simulation parameters
#         self.current_position = Point(x=0.0, y=0.0, z=0.0)
#         self.current_orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
#         self.current_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        
#         # Define test path that passes near the guidance waypoints
#         # Guidance waypoints are: [(5,5,8), (10,5,-5), (15,0,0)]
#         self.test_path = [
#             {'x': 0.0, 'y': 0.0, 'z': 0.0},      # Start
#             {'x': 5.0, 'y': 5.0, 'z': 8.0},      # Near first waypoint
#             {'x': 10.0, 'y': 5.0, 'z': -5.0},    # Near second waypoint
#             {'x': 15.0, 'y': 0.0, 'z': 0.0}      # Near third waypoint
#         ]
#         self.current_segment = 0
#         self.scenario_time = 0.0
#         self.speed = 0.5  # m/s
        
#         # Create timer for publishing test data
#         self.create_timer(0.1, self.timer_callback)  # 10 Hz update rate
        
#         self.get_logger().info('Guidance Test Node started')
    
#     def interpolate_position(self, start, end, alpha):
#         """Interpolate between two positions."""
#         x = start['x'] + (end['x'] - start['x']) * alpha
#         y = start['y'] + (end['y'] - start['y']) * alpha
#         z = start['z'] + (end['z'] - start['z']) * alpha
#         return x, y, z
    
#     def compute_yaw(self, dx, dy):
#         """Compute yaw angle from direction vector."""
#         return math.atan2(dy, dx)
    
#     def compute_pitch(self, dh, dz):
#         """Compute pitch angle from horizontal and vertical distances."""
#         return math.atan2(-dz, dh)
        
#     def timer_callback(self):
#         """Publish test odometry data."""
#         if self.current_segment >= len(self.test_path) - 1:
#             self.get_logger().info('Test complete')
#             rclpy.shutdown()
#             return
            
#         # Get current segment start and end points
#         start = self.test_path[self.current_segment]
#         end = self.test_path[self.current_segment + 1]
        
#         # Calculate segment properties
#         dx = end['x'] - start['x']
#         dy = end['y'] - start['y']
#         dz = end['z'] - start['z']
#         segment_length = math.sqrt(dx**2 + dy**2 + dz**2)
        
#         # Calculate progress along current segment
#         time_for_segment = segment_length / self.speed
#         alpha = self.scenario_time / time_for_segment
        
#         if alpha >= 1.0:
#             # Move to next segment
#             self.current_segment += 1
#             self.scenario_time = 0.0
#             self.get_logger().info(f'Moving to path segment {self.current_segment}')
#             if self.current_segment >= len(self.test_path) - 1:
#                 return
#         else:
#             # Interpolate position
#             x, y, z = self.interpolate_position(start, end, alpha)
#             self.current_position.x = x
#             self.current_position.y = y
#             self.current_position.z = z
            
#             # Calculate orientation
#             self.current_orientation['yaw'] = self.compute_yaw(dx, dy)
#             horizontal_dist = math.sqrt(dx**2 + dy**2)
#             self.current_orientation['pitch'] = self.compute_pitch(horizontal_dist, dz)
            
#             # Set velocity based on direction
#             total_dist = math.sqrt(dx**2 + dy**2 + dz**2)
#             if total_dist > 0:
#                 self.current_velocity.x = (dx / total_dist) * self.speed
#                 self.current_velocity.y = (dy / total_dist) * self.speed
#                 self.current_velocity.z = (dz / total_dist) * self.speed
            
#             # Create and publish odometry message
#             odom_msg = Odometry()
#             odom_msg.header.frame_id = "odom"
#             odom_msg.header.stamp = self.get_clock().now().to_msg()
            
#             # Set position
#             odom_msg.pose.pose.position = self.current_position
            
#             # Convert Euler angles to quaternion
#             quat = euler2quat(
#                 self.current_orientation['roll'],
#                 self.current_orientation['pitch'],
#                 self.current_orientation['yaw']
#             )
#             odom_msg.pose.pose.orientation = Quaternion(
#                 w=quat[0], x=quat[1], y=quat[2], z=quat[3]
#             )
            
#             # Set velocity
#             odom_msg.twist.twist.linear = self.current_velocity
            
#             # Publish message
#             self.odom_pub.publish(odom_msg)
            
#             # Log current position
#             self.get_logger().info(
#                 f"Test Vehicle Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            
#             self.scenario_time += 0.1

# def main(args=None):
#     rclpy.init(args=args)
#     node = GuidanceTestNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()