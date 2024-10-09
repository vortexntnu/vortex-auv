import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import math
from transforms3d.euler import quat2euler


class GuidanceNode(Node):

    def __init__(self):
        super().__init__('guidance_pkg')

        # Publisher to output the surge velocity, pitch angle, and heading angle
        self.output_pub = self.create_publisher(Float32MultiArray,
                                                '/guidance/waypoints', 10)

        # Subscriber to receive odometry data from '/nucleus/odom'
        self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback,
                                 10)

        # Initialize variables
        self.current_position = None
        self.current_velocity = None
        self.current_orientation = None

        # Parameters for the guidance algorithm
        self.U = 1.0  # Desired speed
        self.delta = 1.2  # Lookahead distance

        # Define waypoints as a list of dictionaries with x, y, z coordinates
        self.waypoints = [
            {
                'x': 5.0,
                'y': -5.0,
                'z': -8.0
            },
            {
                'x': 10.0,
                'y': 5.0,
                'z': -5.0
            },
            {
                'x': 15.0,
                'y': 0.0,
                'z': 0.0
            },
            # Add more waypoints as needed
        ]
        self.current_waypoint_index = 0

        # Timer to periodically run the guidance algorithm
        self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        """
        Callback function to process odometry data.
        """
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear
        orientation_q = msg.pose.pose.orientation

        # Convert quaternion to Euler angles
        # ROS quaternion format: (x, y, z, w)
        quaternion = [
            orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z
        ]
        roll, pitch, yaw = quat2euler(quaternion)
        self.current_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    def timer_callback(self):
        """
        Timer callback to compute and publish guidance commands.
        """
        if self.current_position is None or self.current_orientation is None:
            # Wait until odometry data is received
            return

        if self.current_waypoint_index >= len(self.waypoints):
            # All waypoints have been reached
            self.get_logger().info('All waypoints reached.')
            return

        # Get the current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]

        # Extract current position
        current_x = self.current_position.x
        current_y = self.current_position.y
        current_z = self.current_position.z

        # Extract current orientation
        psi = self.current_orientation['yaw']  # Heading angle
        theta = self.current_orientation['pitch']  # Pitch angle

        # Extract target waypoint position
        target_x = waypoint['x']
        target_y = waypoint['y']
        target_z = waypoint['z']

        # Compute distance to the waypoint
        distance = math.sqrt((target_x - current_x)**2 +
                             (target_y - current_y)**2 +
                             (target_z - current_z)**2)

        # Check if the waypoint is reached
        if distance < 0.5:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index}')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.get_logger().info('All waypoints reached.')
                return
            else:
                waypoint = self.waypoints[self.current_waypoint_index]

        # Guidance calculations using a Line-of-Sight (LOS) algorithm
        ref_x = current_x
        ref_y = current_y
        ref_z = current_z

        dx = target_x - ref_x
        dy = target_y - ref_y
        dz = target_z - ref_z

        # Compute desired heading angle (psi_d)
        pi_p = math.atan2(dy, dx)
        pi_p = normalize_angle(pi_p)

        # Compute desired pitch angle (theta_d)
        gamma = math.atan2(-dz, math.hypot(dx, dy))

        # Compute cross-track error
        e_ct = -(dx * (current_y - ref_y) - dy *
                 (current_x - ref_x)) / math.hypot(dx, dy)

        # Compute desired course angle
        course_d = pi_p - math.atan(e_ct / self.delta)
        course_d = normalize_angle(course_d)

        # Set desired surge velocity
        surge_velocity = self.U

        # Set desired heading and pitch angles
        heading_angle = course_d
        pitch_angle = gamma

        # Prepare and publish the output message
        output_msg = Float32MultiArray()
        output_msg.data = [surge_velocity, pitch_angle, heading_angle]
        self.output_pub.publish(output_msg)


def normalize_angle(angle):
    """
    Normalize an angle to the range [-π, π].
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
