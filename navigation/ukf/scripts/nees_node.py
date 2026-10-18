#!/usr/bin/env python3
"""Publishes the NEES of an odometry estimate against a ground truth odometry.

Also publishes the two-sided 95% bounds of the chi-squared distribution that a
consistent filter's NIS and NEES follow, so they can be plotted next to them.

The position NEES uses the position block of the pose covariance, the attitude
NEES uses the rotation block. ROS pose rotation errors are small rotations
about fixed axes in the parent frame, so the attitude error is the world-frame
rotation vector of q_est * q_true^-1.
"""

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from scipy.stats import chi2
from std_msgs.msg import Float64


def nees(error: np.ndarray, covariance: np.ndarray) -> float | None:
    """Normalised estimation error squared, None if the covariance is singular.

    Args:
        error: The estimation error.
        covariance: The covariance the estimator reports for the error.

    Returns:
        e^T P^-1 e, or None if P is not positive definite.
    """
    try:
        return float(error @ np.linalg.solve(covariance, error))
    except np.linalg.LinAlgError:
        return None


class NeesNode(Node):
    def __init__(self):
        super().__init__('nees_node')

        estimate_topic = self.declare_parameter('estimate_topic', 'ukf/odom').value
        truth_topic = self.declare_parameter('truth_topic', 'odom').value
        output_prefix = self.declare_parameter('output_prefix', 'ukf').value

        self.pub_position = self.create_publisher(
            Float64, f'{output_prefix}/nees_position', 10
        )
        self.pub_attitude = self.create_publisher(
            Float64, f'{output_prefix}/nees_attitude', 10
        )

        estimate_sub = message_filters.Subscriber(
            self, Odometry, estimate_topic, qos_profile=qos_profile_sensor_data
        )
        truth_sub = message_filters.Subscriber(
            self, Odometry, truth_topic, qos_profile=qos_profile_sensor_data
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [estimate_sub, truth_sub], queue_size=10, slop=0.1
        )
        self.synchronizer.registerCallback(self.callback)

        # a consistent filter's per sample NIS and NEES are chi-squared with as
        # many degrees of freedom as the quantity has, 3 for position, attitude
        # and the DVL and 1 for depth
        self.bounds = {}
        for dof in (1, 3):
            for side, probability in (('lower', 0.025), ('upper', 0.975)):
                topic = f'{output_prefix}/bounds/chi2_95_{side}_dof{dof}'
                self.bounds[topic] = (
                    self.create_publisher(Float64, topic, 10),
                    float(chi2.ppf(probability, dof)),
                )
        self.create_timer(0.5, self.publish_bounds)

        self.get_logger().info(
            f'NEES of {estimate_topic} against {truth_topic} on '
            f'{output_prefix}/nees_{{position,attitude}}'
        )

    def publish_bounds(self):
        for publisher, value in self.bounds.values():
            publisher.publish(Float64(data=value))

    def callback(self, estimate: Odometry, truth: Odometry):
        p_est = estimate.pose.pose.position
        p_true = truth.pose.pose.position
        position_error = np.array(
            [p_est.x - p_true.x, p_est.y - p_true.y, p_est.z - p_true.z]
        )

        q_est = estimate.pose.pose.orientation
        q_true = truth.pose.pose.orientation
        r_est = Rotation.from_quat([q_est.x, q_est.y, q_est.z, q_est.w])
        r_true = Rotation.from_quat([q_true.x, q_true.y, q_true.z, q_true.w])
        attitude_error = (r_est * r_true.inv()).as_rotvec()

        covariance = np.array(estimate.pose.covariance).reshape(6, 6)

        for publisher, error, block in (
            (self.pub_position, position_error, covariance[0:3, 0:3]),
            (self.pub_attitude, attitude_error, covariance[3:6, 3:6]),
        ):
            value = nees(error, block)
            if value is not None:
                publisher.publish(Float64(data=value))


def main(args=None):
    rclpy.init(args=args)
    node = NeesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
