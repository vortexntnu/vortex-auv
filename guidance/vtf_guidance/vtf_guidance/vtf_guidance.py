#!/usr/bin/env python3

import math
from dataclasses import dataclass

# import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry
from path_calculation import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy import linalg
from std_msgs.msg import Float32MultiArray, Float64
from vortex_msgs.msg import ReferenceFilter


@dataclass
class TargetState:

    x: float = 0.0
    x_dot: float = 0.0
    y: float = 0.0
    y_dot: float = 0.0
    z: float = 0.0
    z_dot: float = 0.0

    def as_array(self):
        return np.array([self.x, self.x_dot, self.y, self.y_dot, self.z, self.z_dot])

    def update(self, new_state: np.ndarray):
        self.x = new_state[0]
        self.x_dot = new_state[1]
        self.y = new_state[2]
        self.y_dot = new_state[3]
        self.z = new_state[4]
        self.z_dot = new_state[5]


@dataclass
class DroneState:

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def pos_as_array(self):
        return np.array([self.x, self.y, self.z])

    def __sub__(self, other):
        return np.array([self.x - other.x, self.y - other.y, self.z - other.z])


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
    """
    Converts Euler angles (roll, pitch, yaw) to a quaternion (w, x, y, z).

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
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.current_position = np.array([0.0, 0.0, 0.0])
        # Initialization
        tstart = 0
        tstop = 25
        increment = 0.1
        self.target_state = TargetState()
        # self.drone_position = np.array([0, 0, 0, 0, 0, 0])
        self.drone_state = DroneState()
        self.x_desired = DroneState()

        self.e_target = 0.0
        self.e_drone = 0.0
        self.r = 1
        self.eta = 1.3
        self.m = 2
        self.Bu = 4

        self.path_index = 0

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

        self.guidance_publisher = self.create_publisher(
            ReferenceFilter, "/dp/reference", 10
        )
        self.timer = self.create_timer(
            0.1, self.publisher_callback
        )  # Timer with 100 ms interval
        self.point_publisher = self.create_publisher(Point, "/dp/vtf_point", 10)

        self.odom_subscriber = self.create_subscription(
            Odometry, "/nucleus/odom", self.odom_subscribe_callback, qos_profile
        )

    def waypoints_subscribe_callback(self, msg: Float64):
        self.get_logger().info("waypoints")

    def odom_subscribe_callback(self, msg: Odometry):
        roll, pitch, yaw = quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

        self.drone_state.x = msg.pose.pose.position.x
        self.drone_state.y = msg.pose.pose.position.y
        self.drone_state.z = msg.pose.pose.position.z
        self.drone_state.roll = roll
        self.drone_state.pitch = pitch
        self.drone_state.yaw = yaw

    def publisher_callback(self):

        msg = ReferenceFilter()
        msg_point = Point()

        path = Path()
        path.generate_G1_path(self.waypoints, 0.2, 0.2)

        # Extract points for simulation in for loop
        points = []
        for segment in path.path:
            for t in np.linspace(0, 1, 100):
                points.append(segment(t))

        points = np.array(points)

        # Get the next waypoint
        goal = np.array(
            [
                points[self.path_index, 0],
                0,
                points[self.path_index, 1],
                0,
                points[self.path_index, 2],
                0,
            ]
        )
        self.e_target = goal - self.target_state.as_array()  # Error vector
        self.e_drone = self.drone_state - self.target_state  # Error vector
        u = np.array(
            [self.e_target[0], self.e_target[2], self.e_target[4]]
        )  # Control input, can be modified if needed

        target_state_next = (
            self.A @ self.target_state.as_array() * self.dt
            + self.target_state.as_array()
            + self.B @ u * self.dt
        )
        self.target_state.update(target_state_next)

        roll = 0.0  # not needed for the tasks
        pitch = np.arctan2(
            self.target_state.z - self.drone_state.z,
            self.target_state.x - self.drone_state.x,
        )

        yaw = np.arctan2(
            self.target_state.y - self.drone_state.y,
            self.target_state.x - self.drone_state.x,
        )
        msg.x = self.target_state.x
        msg.y = self.target_state.y
        msg.z = self.target_state.z
        # q = euler_angle_to_quaternion(self.roll, self.pitch, self.yaw)
        # msg.orientation.w = q[0]
        # msg.orientation.x = q[1]
        # msg.orientation.y = q[2]
        # msg.orientation.z = q[3]
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw

        msg_point.x = self.target_state.x
        msg_point.y = self.target_state.y
        msg_point.z = self.target_state.z
        self.point_publisher.publish(msg_point)

        self.guidance_publisher.publish(msg)
        self.get_logger().info("Send desired states")

        # Move to the next point if virtual target is not too far from the trajectory and there is no big gap between virtual target and drone
        if linalg.norm(self.e_target) < 1 and linalg.norm(self.e_drone) < 1:
            self.path_index += 1

            ## hold the last point


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()

    rclpy.spin(node)

    rclpy.shutdown()

    if __name__ == "__main__":
        main()


if __name__ == "__main__":
    main()
