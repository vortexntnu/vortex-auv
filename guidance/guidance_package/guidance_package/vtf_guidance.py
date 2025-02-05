#!/usr/bin/python3

import math

# import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy import linalg
from std_msgs.msg import Float64

from .path_calculation import Path


def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


def euler_angle_to_quaternion(roll, pitch, yaw):
    """Converts Euler angles (roll, pitch, yaw) to a quaternion (w, x, y, z).

    Args:
        roll: Rotation around the x-axis (in radians).
        pitch: Rotation around the y-axis (in radians).
        yaw: Rotation around the z-axis (in radians).

    Returns:
        w, x, y, z: Quaternion components.
    """
    # Compute the half angles
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Compute quaternion components
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


class GuidanceNode(Node):
    def __init__(self):
        super().__init__("vtf_guidance")
        self.current_position = np.array([0, 0, 0])
        # Initialization
        tstart = 0
        tstop = 25
        increment = 0.1
        self.x_target = np.array([0, 0, 0, 0, 0, 0])
        # self.drone_position = np.array([0, 0, 0, 0, 0, 0])
        self.drone_orientation = np.array([0, 0, 0, 0, 0, 0])
        self.drone_position = np.array([0, 0, 0])
        self.x_desired = np.array([0, 0, 0])
        # self.angles = np.array([0, 0])

        #### states to get from orca
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        position_x = 0
        position_y = 0
        position_z = 0
        ####

        self.i = 0
        self.e_target = 0
        self.e_drone = 0
        self.r = 1
        self.eta = 1.3
        self.m = 2
        self.Bu = 2

        self.i = 0

        self.t = np.arange(tstart, tstop + 1, increment)
        self.dt = 0.1

        Fr = -6 * math.pi * self.r * self.eta / self.m
        self.A = np.array(
            [
                [0, 1, 0, 0, 0, 0],
                [0, Fr, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, Fr, 0, 0],
                [0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, Fr],
            ]
        )
        self.B = np.array(
            [
                [0, self.Bu, 0, 0, 0, 0],
                [0, 0, 0, self.Bu, 0, 0],
                [0, 0, 0, 0, 0, self.Bu],
            ]
        ).transpose()

        self.waypoint_subscriber = self.create_subscription(
            Float64, "waypoints", self.waypoints_subscribe_callback, 10
        )

        # Initialize waypoints here or set up a subscriber
        self.waypoints = [
            [0, 0, 0.5],
            [-2, 4, 2],
            [-4, 4, 4],
            [-6, 5, 4],
            [-8, 3, 4],
            [-6, 1, 4],
            [-4, 2, 4],
            [-2, 2, 2],
            [0, 0, 0.5],
        ]

        # self.dummy_waypoints = np.array([2, 4, -4])

        self.guidance_publisher = self.create_publisher(Pose, "/DP_guidance", 10)
        self.timer = self.create_timer(
            0.1, self.publisher_callback
        )  # Timer with 100 ms interval

        self.odom_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.odom_subscribe_callback, 10
        )

    def waypoints_subscribe_callback(self, msg: Float64):
        self.get_logger().info("waypoints")

    def odom_subscribe_callback(self, msg: Odometry):
        self.drone_orientation = quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

        self.drone_position[0] = msg.pose.pose.position.x
        self.drone_position[1] = msg.pose.pose.position.y
        self.drone_position[2] = msg.pose.pose.position.z

        self.get_logger().info(
            f"Position: x={self.drone_position[0]:.2f}, y={self.drone_position[1]:.2f}, z={self.drone_position[2]:.2f} | Orientation: roll={self.drone_orientation[0]:.2f}, pitch={self.drone_orientation[1]:.2f}, yaw={self.drone_orientation[2]:.2f}"
        )

    def publisher_callback(self):
        msg = Pose()

        path = Path()
        path.generate_G1_path(self.waypoints, 0.2, 0.2)

        # Extract points for simulation in for loop
        points = []
        for segment in path.path:
            for t in np.linspace(0, 1, 100):
                points.append(segment(t))

        points = np.array(points)

        if self.i >= len(points):
            self.get_logger().info("Path complete. Stopping publisher.")
            self.timer.cancel()  # Stop the timer when done
            return

        # Get the next waypoint
        goal = np.array(
            [
                points[self.i, 0],
                0,
                points[self.i, 1],
                0,
                points[self.i, 2],
                0,
            ]
        )
        self.e_target = goal - self.x_target  # Error vector
        self.e_drone = self.x_desired - np.array(
            [self.drone_position[0], self.drone_position[1], self.drone_position[2]]
        )
        u = np.array(
            [self.e_target[0], self.e_target[2], self.e_target[4]]
        )  # Control input, can be modified if needed

        self.x_target = (
            self.A @ self.x_target * self.dt + self.x_target + self.B @ u * self.dt
        )

        self.x_desired = np.array(
            [self.x_target[0], self.x_target[2], self.x_target[4]]
        )

        self.roll = 0  # not needed for the tasks
        self.pitch = np.arctan2(
            self.x_target[4] - self.drone_position[2],
            self.x_target[0] - self.drone_position[0],
        )
        self.yaw = np.arctan2(
            self.x_target[0] - self.drone_position[0],
            self.x_target[2] - self.drone_position[1],
        )
        msg.position.x = self.x_desired[0]
        msg.position.y = self.x_desired[1]
        msg.position.z = self.x_desired[2]
        q = euler_angle_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.orientation.w = q[0]
        msg.orientation.x = q[1]
        msg.orientation.y = q[2]
        msg.orientation.z = q[3]

        self.guidance_publisher.publish(msg)
        self.get_logger().info("Send desired states")

        # Move to the next point if virtual target is not too far from the trajectory and there is no big gap between virtual target and drone
        if linalg.norm(self.e_target) < 1 and linalg.norm(self.e_drone) < 1:
            self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()

    rclpy.spin(node)

    rclpy.shutdown()

    if __name__ == "__main__":
        main()
