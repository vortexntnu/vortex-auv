#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import math

class OdomFormatter(Node):
    def __init__(self):
        super().__init__('odom_formatter')
        self.subscription = self.create_subscription(
            Odometry,
            '/nucleus/odom',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        # Extract position
        pos = msg.pose.pose.position
        # Extract orientation
        q = msg.pose.pose.orientation
        # Convert quaternion to euler angles
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
        
        # Print formatted output
        print("\033[2J\033[H")  # Clear screen and move to top
        print(f"=== Odometry Data ===")
        print(f"Position (x,y,z): {pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}")
        print(f"Orientation (deg):")
        print(f"  Roll : {math.degrees(roll):.2f}")
        print(f"  Pitch: {math.degrees(pitch):.2f}")
        print(f"  Yaw  : {math.degrees(yaw):.2f}")
        print("==================")

def main(args=None):
    rclpy.init(args=args)
    formatter = OdomFormatter()
    rclpy.spin(formatter)
    formatter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
