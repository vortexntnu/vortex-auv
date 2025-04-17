#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ukf_okid import UKF
from ukf_okid_class import MeasModel, StateQuat, process_model


class UKFNode(Node):
    def __init__(self):
        super().__init__("UKFNode")

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # subscribers
        self.dvl_subscriber = self.create_subscription(
            TwistWithCovarianceStamped,
            "/orca/twist",
            self.dvl_callback,
            qos_profile=best_effort_qos,
        )

        self.control_input = self.create_subscription(
            WrenchStamped,
            "/orca/wrench_input",
            self.control_callback,
            qos_profile=best_effort_qos,
        )

        self.odom_publish = self.create_publisher(
            Odometry, "/orca/odometry", qos_profile=best_effort_qos
        )
        dt = self.declare_parameter("dt", 0.01).get_parameter_value().double_value
        self.control_timer = self.create_timer(dt, self.odom_publisher)

        self.current_state = StateQuat()
        x0 = np.zeros(13)
        x0[3] = 1.0  # quaternion: [1, 0, 0, 0]
        P0 = np.eye(12) * 0.5
        self.ukf_model = process_model()
        self.ukf_model.dt = 0.01
        self.ukf_model.mass_interia_matrix = np.array(
            [
                [30.0, 0.0, 0.0, 0.0, 0.0, 0.6],
                [0.0, 30.0, 0.0, 0.0, -0.6, 0.3],
                [0.0, 0.0, 30.0, 0.6, 0.3, 0.0],
                [0.0, 0.0, 0.6, 0.68, 0.0, 0.0],
                [0.0, -0.6, 0.3, 0.0, 3.32, 0.0],
                [0.6, 0.3, 0.0, 0.0, 0.0, 3.34],
            ]
        )
        self.ukf_model.m = 30.0
        self.ukf_model.r_b_bg = np.array([0.01, 0.0, 0.02])
        self.ukf_model.inertia = np.diag([0.68, 3.32, 3.34])
        self.ukf_model.damping_linear = np.array([0.01] * 6)
        # self.ukf_model.added_mass = np.diag([1.0, 1.0, 1.0, 2.0, 2.0, 2.0])

        Q = np.diag([0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        self.ukf = UKF(self.ukf_model, x0, P0, Q)
        self.ukf_flagg = False

    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        # unpack msg
        dvl_measurement = MeasModel()
        # Print received DVL data to console
        # self.get_logger().info(f"DVL data received: x={msg.twist.twist.linear.x}, y={msg.twist.twist.linear.y}, z={msg.twist.twist.linear.z}")
        dvl_measurement.measurement = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        dvl_measurement.covariance = np.array(
            [
                [
                    msg.twist.covariance[0],
                    msg.twist.covariance[1],
                    msg.twist.covariance[3],
                ],
                [
                    msg.twist.covariance[6],
                    msg.twist.covariance[7],
                    msg.twist.covariance[8],
                ],
                [
                    msg.twist.covariance[12],
                    msg.twist.covariance[13],
                    msg.twist.covariance[14],
                ],
            ]
        )

        self.ukf.measurement_update(self.current_state, dvl_measurement)
        self.current_state = self.ukf.posteriori_estimate(
            self.current_state, dvl_measurement
        )
        self.ukf_flagg = True

    def control_callback(self, msg: WrenchStamped):
        # unpack message
        control_array = np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ]
        )
        self.ukf_model.Control_input = control_array

    def odom_publisher(self):
        msg = Odometry()

        if self.ukf_flagg == False:
            self.current_state = self.ukf.unscented_transform(self.current_state)
        else:
            self.ukf_flagg = False

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.current_state.position[0]
        msg.pose.pose.position.y = self.current_state.position[1]
        msg.pose.pose.position.z = self.current_state.position[2]
        msg.pose.pose.orientation.w = self.current_state.orientation[0]
        msg.pose.pose.orientation.x = self.current_state.orientation[1]
        msg.pose.pose.orientation.y = self.current_state.orientation[2]
        msg.pose.pose.orientation.z = self.current_state.orientation[3]
        msg.twist.twist.linear.x = self.current_state.velocity[0]
        msg.twist.twist.linear.y = self.current_state.velocity[1]
        msg.twist.twist.linear.z = self.current_state.velocity[2]
        msg.twist.twist.angular.x = self.current_state.angular_velocity[0]
        msg.twist.twist.angular.y = self.current_state.angular_velocity[1]
        msg.twist.twist.angular.z = self.current_state.angular_velocity[2]

        self.odom_publish.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ukf_node = UKFNode()
    rclpy.spin(ukf_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
