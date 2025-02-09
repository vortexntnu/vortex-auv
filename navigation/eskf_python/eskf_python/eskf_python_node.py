#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

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
        super().__init__("eskf_python_node")

        # This callback will supply information from the IMU (Inertial Measurement Unit) 1000 Hz
        # TEMPORARILY ADDED FOR TESTING
        self.imu_subscriber_ = self.create_subscription(
            Odometry, '/orca/imu', self.state_callback, qos_profile=qos_profile
        )

        # This publisher will publish the estimtaed state of the vehicle
        self.state_publisher_ = self.create_publisher(
            Odometry, '/orca/odom', qos_profile=qos_profile
        )

        self.eskf_modual = ErrorStateKalmanFilter()
        self.current_state_nom = StateVector_quaternion()
        self.current_state_error = StateVector_euler()
        self.measurement_pred = MeasurementModel()

        self.get_logger().info("hybridpath_controller_node started")

    def imu_callback(self, msg: Odometry):
        self.get_logger().info(f"Received IMU message: {msg}")

        # Get the IMU data

        # SOME CONVERSION HERE TO SUITABLE TYPE
        imu_data = "something"

        # Update the filter with the IMU data
        self.current_state_nom, self.current_state_error = (
            ErrorStateKalmanFilter.imu_update_states(
                self.current_state_nom, self.current_state_error, imu_data
            )
        )

        # Publish the estimated state
        """
        Some conversion function from the custom state type to odometry message
        This needs to be worked on
        """

    def filter_callback(self):
        """Callback function for the filter measurement update,
        this will be called when the filter needs to be updated with the DVL data.
        """
        self.get_logger().info("Filter callback, got DVL data")

        # Get the DVL data
        dvl_data = "something"

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

        # Publish the estimated state
        """
        Some conversion function from the custom state type to odometry message
        This needs to be worked on
        """


def main(args=None):
    rclpy.init(args=args)
    node = ESKalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
