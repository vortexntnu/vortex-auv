#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from velocity_controller_lqr.velocity_controller_lqr_lib import LQRController
from vortex_msgs.msg import LOSGuidance


class LinearQuadraticRegulator(Node):
    def __init__(self):
        super().__init__("velocity_controller_lqr_node")

        self.declare_parameter("odom_path", "/nucleus/odom")
        self.declare_parameter("guidance_path", "/guidance/los")
        self.declare_parameter("thrust_path", "/thrust/wrench_input")

        odom_path = self.get_parameter("odom_path").value
        guidance_path = self.get_parameter("guidance_path").value
        thrust_path = self.get_parameter("thrust_path").value

        # SUBSRCIBERS -----------------------------------
        self.nucleus_subscriber = self.create_subscription(
            Odometry, odom_path, self.nucleus_callback, 20
        )
        self.guidance_subscriber = self.create_subscription(
            LOSGuidance, guidance_path, self.guidance_callback, 20
        )

        # PUBLISHERS ------------------------------------
        self.publisherLQR = self.create_publisher(Wrench, thrust_path, 10)

        # TIMERS ----------------------------------------
        self.control_timer = self.create_timer(0.1, self.controller)

        # ROS2 SHENNANIGANS with parameters
        self.declare_parameter("max_force", 99.5)
        max_force = self.get_parameter("max_force").value

        self.declare_parameter("q_surge", 75)
        self.declare_parameter("q_pitch", 175)
        self.declare_parameter("q_yaw", 175)

        self.declare_parameter("r_surge", 0.3)
        self.declare_parameter("r_pitch", 0.4)
        self.declare_parameter("r_yaw", 0.4)

        self.declare_parameter("i_surge", 0.3)
        self.declare_parameter("i_pitch", 0.4)
        self.declare_parameter("i_yaw", 0.3)

        q_surge = self.get_parameter("q_surge").value
        q_pitch = self.get_parameter("q_pitch").value
        q_yaw = self.get_parameter("q_yaw").value

        r_surge = self.get_parameter("r_surge").value
        r_pitch = self.get_parameter("r_pitch").value
        r_yaw = self.get_parameter("r_yaw").value

        i_surge = self.get_parameter("i_surge").value
        i_pitch = self.get_parameter("i_pitch").value
        i_yaw = self.get_parameter("i_yaw").value

        self.controller = LQRController(
            q_surge,
            q_pitch,
            q_yaw,
            r_surge,
            r_pitch,
            r_yaw,
            i_surge,
            i_pitch,
            i_yaw,
            max_force,
        )

        # Only keeping variables that need to be updated inside of a callback
        self.C = np.zeros((3, 3))  # Initialize Coriolis matrix
        self.states = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )  # State vector 1. surge, 2. pitch, 3. yaw, 4. integral_surge, 5. integral_pitch, 6. integral_yaw
        self.guidance_values = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )  # Array to store guidance values

    # ---------------------------------------------------------------Callback Functions---------------------------------------------------------------

    def nucleus_callback(
        self, msg: Odometry
    ):  # callback function to read data from nucleus
        dummy, self.states[1], self.states[2] = LQRController.quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

        self.states[0] = msg.twist.twist.linear.x

        self.C = self.controller.calculate_coriolis_matrix(
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )  # coriolis matrix

    def guidance_callback(
        self, msg: LOSGuidance
    ):  # Function to read data from guidance
        self.guidance_values[0] = msg.surge
        self.guidance_values[1] = msg.pitch
        self.guidance_values[2] = msg.yaw

    # ---------------------------------------------------------------Publisher Functions---------------------------------------------------------------

    def controller(self):  # The LQR controller function
        msg = Wrench()

        u = self.controller.calculate_lqr_u(self.C, self.states, self.guidance_values)
        msg.force.x = u[0]
        msg.torque.y = u[1]
        msg.torque.z = u[2]

        self.get_logger().info(
            f"Input values: {msg.force.x}, {msg.torque.y}, {msg.torque.z}"
        )

        # Publish the control input
        self.publisherLQR.publish(msg)


# ---------------------------------------------------------------Main Function---------------------------------------------------------------


def main(args=None):  # main function
    rclpy.init(args=args)
    node = LinearQuadraticRegulator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# Anders er goated
