#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu,
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped

# NEED TO CHANGE THIS TO THE CORRECT PATH
from eskf_python.eskf_python_filter import (
    ErrorStateKalmanFilter,
    MeasurementModel,
    StateVector_euler,
    StateVector_quaternion,
)

qos_profile = QoSProfile(
    depth=1,
    history=qos_profile_sensor_data.history,
    reliability=qos_profile_sensor_data.reliability,
)


class ESKalmanFilterNode(Node):
    def __init__(self):
        super().__init__("sp_ukf_python_node")

        # This callback will supply information from the IMU (Inertial Measurement Unit) 1000 Hz
        self.imu_subscriber_ = self.create_subscription(
            Imu, '/orca/imu', self.imu_callback, qos_profile=qos_profile
        )

        self.twist_dvl_subscriber_ = self.create_subscription(
            TwistWithCovarianceStamped, '/dvl/twist', self.filter_callback, qos_profile=qos_profile
        )

        # This publisher will publish the estimtaed state of the vehicle
        self.state_publisher_ = self.create_publisher(
            Odometry, '/orca/odom', qos_profile=qos_profile
        )

        self.eskf_modual = ErrorStateKalmanFilter()
        self.current_state_nom = StateVector_quaternion()
        self.current_state_error = StateVector_euler()
        self.measurement_pred = MeasurementModel()
        self.odom_msg = Odometry()

        self.get_logger().info("Unscented Kalman Filter started")

    def imu_callback(self, msg: Imu):

        # Get the IMU data

        imu_acceleartion = msg.linear_acceleration
        imu_angular_velocity = msg.angular_velocity

        # Combine the IMU data
        imu_data = np.array([imu_acceleartion.x, imu_acceleartion.y, imu_acceleartion.z, imu_angular_velocity.x, imu_angular_velocity.y, imu_angular_velocity.z])

        # Update the filter with the IMU data
        self.current_state_nom, self.current_state_error = (
            ErrorStateKalmanFilter.imu_update_states(
                self.current_state_nom, self.current_state_error, imu_data
            )
        )

        # Inserting the nominal state into the msg
        self.odom_msg.pose.pose.position.x = self.current_state_nom.position[0]
        self.odom_msg.pose.pose.position.y = self.current_state_nom.position[1]
        self.odom_msg.pose.pose.position.z = self.current_state_nom.position[2]
        self.odom_msg.pose.pose.orientation.x = self.current_state_nom.orientation[0]
        self.odom_msg.pose.pose.orientation.y = self.current_state_nom.orientation[1]
        self.odom_msg.pose.pose.orientation.z = self.current_state_nom.orientation[2]
        self.odom_msg.pose.pose.orientation.w = self.current_state_nom.orientation[3]
        self.odom_msg.twist.twist.linear.x = self.current_state_nom.velocity[0]
        self.odom_msg.twist.twist.linear.y = self.current_state_nom.velocity[1]
        self.odom_msg.twist.twist.linear.z = self.current_state_nom.velocity[2]
        self.odom_msg.twist.twist.angular.x = imu_angular_velocity.x
        self.odom_msg.twist.twist.angular.y = imu_angular_velocity.y
        self.odom_msg.twist.twist.angular.z = imu_angular_velocity.z

        # Publish
        self.state_publisher_.publish(self.odom_msg)



    def filter_callback(self, msg: TwistWithCovarianceStamped):
        """Callback function for the filter measurement update,
        this will be called when the filter needs to be updated with the DVL data.
        """
        self.get_logger().info("Filter callback, got DVL data")

        # Get the DVL data (linear velocity)
        dvl_data = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

        # Update the filter with the DVL data
        self.current_state_nom, self.current_state_error = (
            ErrorStateKalmanFilter.dvl_update_states(
                self.current_state_nom, self.current_state_error, dvl_data
            )
        )
        self.current_state_nom, self.current_state_error = (
            ErrorStateKalmanFilter.injection_and_reset(
                self.current_state_nom, self.current_state_error
            )
        )

        # Inserting data into the msg
        self.odom_msg.pose.pose.position.x = self.current_state_nom.position[0]
        self.odom_msg.pose.pose.position.y = self.current_state_nom.position[1]
        self.odom_msg.pose.pose.position.z = self.current_state_nom.position[2]
        self.odom_msg.pose.pose.orientation.x = self.current_state_nom.orientation[0]
        self.odom_msg.pose.pose.orientation.y = self.current_state_nom.orientation[1]
        self.odom_msg.pose.pose.orientation.z = self.current_state_nom.orientation[2]
        self.odom_msg.pose.pose.orientation.w = self.current_state_nom.orientation[3]
        self.odom_msg.twist.twist.linear.x = self.current_state_nom.velocity[0]
        self.odom_msg.twist.twist.linear.y = self.current_state_nom.velocity[1]
        self.odom_msg.twist.twist.linear.z = self.current_state_nom.velocity[2]
        self.odom_msg.twist.twist.linear.z = self.current_state_nom.velocity[2]

        # Publishing the data
        self.state_publisher_.publish(self.odom_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ESKalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
