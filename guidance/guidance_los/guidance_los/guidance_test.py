#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from vortex_msgs.msg import LOSGuidance
import numpy as np
from transforms3d.euler import euler2quat, quat2euler
import math


class GuidanceLOSTestNode(Node):

    def __init__(self):
        super().__init__('guidance_los_test')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/nucleus/odom', 10)

        # Subscriber for guidance commands
        self.create_subscription(LOSGuidance, '/guidance/los',
                                 self.guidance_callback, 10)

        # Test scenarios - each tests different aspects of guidance
        self.test_scenarios = [{
            'name': 'Forward motion with depth change',
            'x': 10.0,
            'y': 0.0,
            'z': -5.0
        }, {
            'name': 'Combined horizontal and vertical motion',
            'x': 15.0,
            'y': 10.0,
            'z': -3.0
        }, {
            'name': 'Pure horizontal motion',
            'x': 20.0,
            'y': 15.0,
            'z': -3.0
        }, {
            'name': 'Return to origin with depth change',
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }]

        # Vehicle state
        self.state = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'u': 0.0,  # surge velocity
            'v': 0.0,  # sway velocity
            'w': 0.0  # heave velocity
        }

        # Control inputs from guidance
        self.guidance_commands = {'surge': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        # Simulation parameters
        self.dt = 0.1  # 10 Hz
        self.time = 0.0
        self.last_log_time = 0.0

        # Vehicle dynamics parameters
        self.tau = 1.0  # Time constant for velocity response
        self.max_pitch_rate = 0.2  # rad/s
        self.max_yaw_rate = 0.3  # rad/s

        # Create timer
        self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('LOS Guidance Test Node Started')
        self.get_logger().info('Testing 3-DOF guidance behavior...')

    def guidance_callback(self, msg):
        """Receive guidance commands."""
        self.guidance_commands['surge'] = msg.surge
        self.guidance_commands['pitch'] = msg.pitch
        self.guidance_commands['yaw'] = msg.yaw

    def update_vehicle_state(self):
        """Simple vehicle dynamics simulation."""
        # Update velocities with first-order response
        target_surge = self.guidance_commands['surge']
        self.state['u'] += (target_surge -
                            self.state['u']) * self.dt / self.tau

        # Update angles with rate limiting
        pitch_error = self.guidance_commands['pitch'] - self.state['pitch']
        yaw_error = self.normalize_angle(self.guidance_commands['yaw'] -
                                         self.state['yaw'])

        self.state['pitch'] += np.clip(pitch_error,
                                       -self.max_pitch_rate * self.dt,
                                       self.max_pitch_rate * self.dt)
        self.state['yaw'] += np.clip(yaw_error, -self.max_yaw_rate * self.dt,
                                     self.max_yaw_rate * self.dt)

        # Update position based on velocities and orientation
        speed = self.state['u']
        yaw = self.state['yaw']
        pitch = self.state['pitch']

        # Calculate velocity components
        vx = speed * math.cos(pitch) * math.cos(yaw)
        vy = speed * math.cos(pitch) * math.sin(yaw)
        vz = -speed * math.sin(
            pitch)  # Negative because positive pitch causes negative z motion

        # Update position
        self.state['x'] += vx * self.dt
        self.state['y'] += vy * self.dt
        self.state['z'] += vz * self.dt

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def timer_callback(self):
        """Simulate vehicle motion and publish odometry."""
        self.time += self.dt

        # Update vehicle state
        self.update_vehicle_state()

        # Create and publish odometry message
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()

        # Set position
        odom.pose.pose.position = Point(x=float(self.state['x']),
                                        y=float(self.state['y']),
                                        z=float(self.state['z']))

        # Set orientation
        quat = euler2quat(self.state['roll'], self.state['pitch'],
                          self.state['yaw'])
        odom.pose.pose.orientation = Quaternion(w=float(quat[0]),
                                                x=float(quat[1]),
                                                y=float(quat[2]),
                                                z=float(quat[3]))

        # Set velocities
        odom.twist.twist.linear = Vector3(x=float(self.state['u']),
                                          y=float(self.state['v']),
                                          z=float(self.state['w']))

        self.odom_pub.publish(odom)

        # Log detailed information every second
        if self.time - self.last_log_time >= 1.0:
            self.last_log_time = self.time

            self.get_logger().info(
                f"\n=== LOS Guidance Test Status ===\n"
                f"Time: {self.time:.1f}s\n"
                f"\nPosition:\n"
                f"  X: {self.state['x']:.2f}m\n"
                f"  Y: {self.state['y']:.2f}m\n"
                f"  Z: {self.state['z']:.2f}m\n"
                f"\nOrientation:\n"
                f"  Pitch: {math.degrees(self.state['pitch']):.1f}°\n"
                f"  Yaw: {math.degrees(self.state['yaw']):.1f}°\n"
                f"\nVelocities:\n"
                f"  Surge: {self.state['u']:.2f} m/s\n"
                f"\nGuidance Commands:\n"
                f"  Surge: {self.guidance_commands['surge']:.2f} m/s\n"
                f"  Pitch: {math.degrees(self.guidance_commands['pitch']):.1f}°\n"
                f"  Yaw: {math.degrees(self.guidance_commands['yaw']):.1f}°\n")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GuidanceLOSTestNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
