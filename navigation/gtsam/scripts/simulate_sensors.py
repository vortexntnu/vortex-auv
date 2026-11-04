#!/usr/bin/env python3
"""Publish synthetic STIM300/Nucleus measurements, a simulation clock and truth."""

import math
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_model import (
    DVL_OFFSET,
    DVL_YAW,
    IMU_OFFSET,
    SensorModel,
    bottom_lock_available,
    ideal_measurements,
    quaternion_from_rotation,
    trajectory,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster


class SensorSimulator(Node):
    """Use a fixed simulation timestep so each seed produces identical samples."""

    def __init__(self):
        super().__init__("gtsam_sensor_simulator")
        self.kind = self.declare_parameter("trajectory", "turn").value
        self.duration = self.declare_parameter(
            "duration", 100.0 if self.kind == "square_barrel_roll" else 60.0
        ).value
        self.rate = self.declare_parameter("imu_rate", 1000.0).value
        self.dvl_rate = self.declare_parameter("dvl_rate", 8.0).value
        self.dropout_start = self.declare_parameter(
            "dropout_start", -1.0 if self.kind == "square_barrel_roll" else 20.0
        ).value
        self.dropout_end = self.declare_parameter(
            "dropout_end", -1.0 if self.kind == "square_barrel_roll" else 25.0
        ).value
        self.dvl_max_tilt_deg = self.declare_parameter("dvl_max_tilt_deg", 30.0).value
        bottom_lock_available(np.eye(3), self.dvl_max_tilt_deg)  # Validate at startup.
        prefix = self.declare_parameter("frame_prefix", "nautilus").value
        self.prefix = prefix + "/" if prefix else ""
        self.model = SensorModel(
            seed=self.declare_parameter("seed", 42).value,
            profile=self.declare_parameter(
                "imu_profile", "stim300_10g_provisional"
            ).value,
            noise=self.declare_parameter("noise", True).value,
            stress_scale=self.declare_parameter("stress_scale", 1.0).value,
        )
        if (
            self.rate <= 0
            or self.dvl_rate <= 0
            or self.dvl_rate > self.rate
            or self.duration <= 0
        ):
            raise ValueError("Invalid simulation rates/duration")
        trajectory(0.0, self.kind)  # Validate before starting timers.
        self.imu_pub = self.create_publisher(
            Imu, "imu/data_raw", qos_profile_sensor_data
        )
        self.dvl_pub = self.create_publisher(
            TwistWithCovarianceStamped, "dvl/twist", qos_profile_sensor_data
        )
        self.lock_pub = self.create_publisher(
            Bool,
            "dvl/bottom_lock",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.truth_pub = self.create_publisher(
            Odometry, "gtsam/truth", qos_profile_sensor_data
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for child, offset, yaw in (
            ("imu_link", IMU_OFFSET, 0.0),
            ("dvl_link", DVL_OFFSET, DVL_YAW),
        ):
            transform = TransformStamped()
            transform.header.frame_id = self.prefix + "base_link"
            transform.child_frame_id = self.prefix + child
            transform.transform.translation.x = float(offset[0])
            transform.transform.translation.y = float(offset[1])
            transform.transform.translation.z = float(offset[2])
            transform.transform.rotation.z = math.sin(yaw / 2)
            transform.transform.rotation.w = math.cos(yaw / 2)
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        self.create_subscription(
            Odometry, "gtsam/odom", self.on_estimate, qos_profile_sensor_data
        )
        self.errors = []
        self.index = 0
        self.finished = False
        self.next_dvl = 0.0
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def on_estimate(self, msg):
        """Compare at the estimate's timestamp; truth never feeds the estimator."""
        time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - 10.0
        p, v, _, rotation, omega, _ = trajectory(time, self.kind)
        p = p + rotation @ IMU_OFFSET - IMU_OFFSET
        v = v + rotation @ np.cross(omega, IMU_OFFSET)
        measured_p = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        measured_v = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        q = msg.pose.pose.orientation
        measured_q = np.array([q.x, q.y, q.z, q.w])
        measured_q /= np.linalg.norm(measured_q)
        dot = abs(float(measured_q @ quaternion_from_rotation(rotation)))
        orientation_error = 2 * math.acos(min(1.0, dot))
        self.errors.append(
            [
                float(np.linalg.norm(measured_p - p)),
                float(np.linalg.norm(measured_v - rotation.T @ v)),
                orientation_error,
            ]
        )

    def tick(self):
        """Generate one fixed-time sensor sample."""
        time = self.index / self.rate
        if time > self.duration:
            self.timer.cancel()
            self.finished = True
            if self.errors:
                rmse = np.sqrt(np.mean(np.square(self.errors), axis=0))
                self.get_logger().info(
                    f"RMSE: position={rmse[0]:.6f} m, IMU velocity={rmse[1]:.6f} m/s, "
                    f"orientation={math.degrees(rmse[2]):.6f} deg; {len(self.errors)} estimates"
                )
            else:
                self.get_logger().error("Simulation finished without estimator output")
            return
        stamp = rclpy.time.Time(nanoseconds=round((10.0 + time) * 1e9)).to_msg()
        clock = Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)
        state = trajectory(time, self.kind)
        acceleration, omega, dvl = ideal_measurements(state)
        acceleration, omega = self.model.imu(acceleration, omega, 1.0 / self.rate)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.prefix + "imu_link"
        imu.orientation_covariance[0] = -1.0
        (
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ) = map(float, acceleration)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(
            float, omega
        )
        for diagonal in (0, 4, 8):
            imu.linear_acceleration_covariance[diagonal] = (
                self.model.accel_density**2 * self.rate
            )
            imu.angular_velocity_covariance[diagonal] = (
                self.model.gyro_density**2 * self.rate
            )
        self.imu_pub.publish(imu)
        if time + 1e-9 >= self.next_dvl:
            self.next_dvl += 1.0 / self.dvl_rate
            has_lock = bottom_lock_available(state[3], self.dvl_max_tilt_deg)
            has_lock = has_lock and not self.dropout_start <= time < self.dropout_end
            self.lock_pub.publish(Bool(data=has_lock))
            if has_lock:
                msg = TwistWithCovarianceStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = self.prefix + "dvl_link"
                (
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ) = map(float, self.model.dvl(dvl))
                for diagonal in (0, 7, 14):
                    msg.twist.covariance[diagonal] = 0.005**2
                self.dvl_pub.publish(msg)
        truth = Odometry()
        truth.header.stamp = stamp
        truth.header.frame_id = self.prefix + "odom"
        truth.child_frame_id = self.prefix + "imu_link"
        p, v, _, rotation, w, _ = state
        p = p + rotation @ IMU_OFFSET - IMU_OFFSET
        v = v + rotation @ np.cross(w, IMU_OFFSET)
        (
            truth.pose.pose.position.x,
            truth.pose.pose.position.y,
            truth.pose.pose.position.z,
        ) = map(float, p)
        (
            truth.pose.pose.orientation.x,
            truth.pose.pose.orientation.y,
            truth.pose.pose.orientation.z,
            truth.pose.pose.orientation.w,
        ) = map(float, quaternion_from_rotation(rotation))
        (
            truth.twist.twist.linear.x,
            truth.twist.twist.linear.y,
            truth.twist.twist.linear.z,
        ) = map(float, rotation.T @ v)
        (
            truth.twist.twist.angular.x,
            truth.twist.twist.angular.y,
            truth.twist.twist.angular.z,
        ) = map(float, w)
        self.truth_pub.publish(truth)
        self.index += 1


def main():
    """Run for the configured duration and print the final comparison metrics."""
    rclpy.init()
    node = SensorSimulator()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may forward a second SIGINT while entities are destroyed.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
