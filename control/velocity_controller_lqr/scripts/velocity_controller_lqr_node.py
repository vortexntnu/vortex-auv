#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    WrenchStamped,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from velocity_controller_lqr.velocity_controller_lqr_lib import (
    GuidanceValues,
    LQRController,
    LQRParameters,
    State,
)
from vortex_msgs.msg import LOSGuidance


class LinearQuadraticRegulator(Node):
    def __init__(self):
        super().__init__("velocity_controller_lqr_node")

        self.get_topics()

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------------------- SUBSCRIBERS ---------------------------

        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            qos_profile=best_effort_qos,
        )

        self.twist_subscriber = self.create_subscription(
            TwistWithCovarianceStamped,
            self.twist_topic,
            self.twist_callback,
            qos_profile=best_effort_qos,
        )

        self.operationmode_subscriber = self.create_subscription(
            String,
            self.operation_mode_topic,
            self.operation_callback,
            qos_profile=2,
        )
        self.killswitch_subscriber = self.create_subscription(
            Bool,
            self.killswitch_topic,
            self.killswitch_callback,
            qos_profile=2,
        )

        self.guidance_subscriber = self.create_subscription(
            LOSGuidance,
            self.los_topic,
            self.guidance_callback,
            qos_profile=best_effort_qos,
        )

        # ---------------------------- PUBLISHERS ----------------------------
        self.publisherLQR = self.create_publisher(
            WrenchStamped, self.wrench_input_topic, best_effort_qos
        )

        # ------------------------------ TIMERS ------------------------------
        dt = self.declare_parameter("dt", 0.1).get_parameter_value().double_value
        self.control_timer = self.create_timer(dt, self.control_loop)

        # ------------------ ROS2 PARAMETERS AND CONTROLLER ------------------
        self.lqr_params = LQRParameters()
        inertia_matrix = self.get_parameters()
        self.controller = LQRController(self.lqr_params, inertia_matrix)

        # ---------------- CALLBACK VARIABLES INITIALIZATION -----------------
        self.coriolis_matrix = np.zeros((3, 3))
        self.states = State()
        self.guidance_values = GuidanceValues()

    def get_topics(self):
        """Get the topics from the parameter file."""
        topics = [
            "pose",
            "twist",
            "los",
            "wrench_input",
            "operation_mode",
            "killswitch",
        ]
        for topic in topics:
            if topic == "los":
                self.declare_parameter("topics.guidance." + topic, "_")
                setattr(
                    self,
                    topic + "_topic",
                    self.get_parameter("topics.guidance." + topic).value,
                )
                continue
            self.declare_parameter("topics." + topic, "_")
            setattr(
                self,
                topic + "_topic",
                self.get_parameter("topics." + topic).value,
            )

    def get_parameters(self):
        """Updates the LQR_params in the LQR_parameters Dataclass, and gets the inertia matrix from config.

        Returns:
        inertia_matrix: np.array: The inertia matrix of the AUV
        """
        self.declare_parameter("LQR_params.q_surge", 75)
        self.declare_parameter("LQR_params.q_pitch", 175)
        self.declare_parameter("LQR_params.q_yaw", 175)

        self.declare_parameter("LQR_params.r_surge", 0.3)
        self.declare_parameter("LQR_params.r_pitch", 0.4)
        self.declare_parameter("LQR_params.r_yaw", 0.4)

        self.declare_parameter("LQR_params.i_surge", 0.3)
        self.declare_parameter("LQR_params.i_pitch", 0.4)
        self.declare_parameter("LQR_params.i_yaw", 0.3)

        self.declare_parameter("LQR_params.i_weight", 0.5)

        self.declare_parameter("max_force", 99.5)
        self.declare_parameter(
            "inertia_matrix", [30.0, 0.6, 0.0, 0.6, 1.629, 0.0, 0.0, 0.0, 1.729]
        )

        self.lqr_params.q_surge = self.get_parameter("LQR_params.q_surge").value
        self.lqr_params.q_pitch = self.get_parameter("LQR_params.q_pitch").value
        self.lqr_params.q_yaw = self.get_parameter("LQR_params.q_yaw").value

        self.lqr_params.r_surge = self.get_parameter("LQR_params.r_surge").value
        self.lqr_params.r_pitch = self.get_parameter("LQR_params.r_pitch").value
        self.lqr_params.r_yaw = self.get_parameter("LQR_params.r_yaw").value

        self.lqr_params.i_surge = self.get_parameter("LQR_params.i_surge").value
        self.lqr_params.i_pitch = self.get_parameter("LQR_params.i_pitch").value
        self.lqr_params.i_yaw = self.get_parameter("LQR_params.i_yaw").value

        self.lqr_params.i_weight = self.get_parameter("LQR_params.i_weight").value
        self.lqr_params.max_force = self.get_parameter("max_force").value

        inertia_matrix_flat = self.get_parameter("inertia_matrix").value
        inertia_matrix = np.array(inertia_matrix_flat).reshape((3, 3))

        return inertia_matrix

    # ---------------------------------------------------------------CALLBACK FUNCTIONS---------------------------------------------------------------

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback function for the pose data from DVL.

        Parameters: msg: PoseWithCovarianceStamped The pose data from the DVL.

        """
        _, self.states.pitch, self.states.yaw = LQRController.quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

    def operation_callback(self, msg: String):
        """Callback function for the operation mode data.

        Parameters: String: msg: The operation mode data from the AUV.

        """
        self.controller.operation_mode = msg.data

    def twist_callback(self, msg: TwistWithCovarianceStamped):
        """Callback function for the Twist data from DVL.

        Parameters: msg: TwistWithCovarianceStamped The twist data from the DVL.

        """
        self.states.surge = msg.twist.twist.linear.x

        self.coriolis_matrix = LQRController.calculate_coriolis_matrix(
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )

    def guidance_callback(self, msg: LOSGuidance):
        """Callback function for the guidance data.

        Parameters: LOSGuidance: msg: The guidance data from the LOS guidance system.

        """
        self.guidance_values.surge = msg.surge
        self.guidance_values.pitch = msg.pitch
        self.guidance_values.yaw = msg.yaw

    def killswitch_callback(self, msg: Bool):
        """Callback function for the killswitch data.

        Parameters: String: msg: The killswitch data from the AUV.

        """
        self.controller.killswitch = msg.data
        if self.controller.killswitch:
            self.controller.reset_controller()

    # ---------------------------------------------------------------PUBLISHER FUNCTIONS-------------------------------------------------------------

    def control_loop(self):
        """The control loop that calculates the input for the LQR controller."""
        msg = WrenchStamped()

        u = self.controller.calculate_lqr_u(
            self.coriolis_matrix, self.states, self.guidance_values
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = float(u[0])
        msg.wrench.torque.y = float(u[1])
        msg.wrench.torque.z = float(u[2])

        if (
            self.controller.killswitch == False
            and self.controller.operation_mode == "autonomous mode"
        ):
            self.publisherLQR.publish(msg)

        else:
            self.controller.reset_controller()


# ----------------------------------------------------------------------MAIN FUNCTION----------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    lqr_node = LinearQuadraticRegulator()
    executor = MultiThreadedExecutor()
    executor.add_node(lqr_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        lqr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣠⢴⣤⣠⣤⢤⣂⣔⣲⣒⣖⡺⢯⣝⡿⣿⣿⣿⣷⣶⣶⣢⢦⣤⣄⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣯⣿⣾⣿⣶⣺⣯⡿⣓⣽⢞⡸⣻⢏⣋⠌⣛⣭⣿⢟⣿⣛⣿⢷⣿⣿⣿⡟⣿⣻⣵⣲⢢⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⠀⠀⢀⣀⣤⡴⠲⠶⢦⠤⢤⡤⠿⠿⠿⠿⣿⣽⣿⣽⣷⣿⢽⣾⣷⣭⡞⣩⡐⠏⡋⣽⡬⣭⠏⢍⣞⢿⣽⣿⣷⢿⣿⣿⡿⠾⣿⢶⡶⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⣤⣖⠯⡙⢢⡁⠄⢢⡤⢠⢀⣸⠀⣄⡠⣀⣀⣦⡄⣠⢀⡃⠽⣽⠬⠍⣿⣿⣤⣥⣷⣿⣿⣿⣾⡍⠛⢌⠈⢫⣍⢋⣍⡁⠹⢍⠈⣳⢎⠴⠟⠻⢧⣄⠀⠀⠀⠀⠀
# ⠀⠀⣠⣾⣿⣿⣿⣯⡔⠆⠠⠈⣿⣿⠾⡾⠿⣶⠿⡟⣻⡛⣭⢙⠍⢩ANDERS ER GOATED⣤⣥⣩⣶⣟⣻⠧⣻⠶⢤⢰⡱⣬⣤⣌⣑⠞⠲⠓⠭⡀⠀⠀⠀
# ⠀⠐⣿⣿⣿⢟⡋⢈⢤⣤⣷⢿⣿⠒⢜⣁⡱⡧⢿⣹⣷⣿⡿⣷⠌⣢⣟⢱⢽⡨⢊⠴⡉⢉⡿⣯⢿⣏⢹⠏⣯⣩⣙⠾⢿⣳⣶⢻⣟⣿⠧⢀⠋⠟⣿⡧⠠⠄⡤⠈⢠⠀⠀
# ⠀⠀⠘⠻⠿⠶⠶⠿⠛⣹⣟⡞⠸⣬⠐⠙⢍⢉⣔⠪⠟⡛⠫⢵⣾⣣⣼⣽⢈⠔⡅⣜⡽⢯⢞⡕⡠⠓⣡⢚⣷⣷⣿⣳⡄⠢⠉⠛⢿⣲⢿⣶⢿⣬⣾⣛⠳⣼⡮⠳⡂⠒⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⢠⠏⡁⢉⣀⣑⣆⡐⠊⣅⡕⢦⣀⣱⡶⢫⣨⢟⠽⠕⣇⢶⣵⣋⢝⣉⣋⠜⠉⠉⡯⠛⣿⣿⣿⣾⣳⠠⠤⠪⠁⠊⠉⠻⣟⣾⣿⣿⣟⣧⣧⢸⠂⠠⢠
# ⠀⠀⠀⠀⠀⠀⠀⣠⣾⢞⢉⠿⢿⣟⡓⠖⢾⡏⢠⣾⣿⠛⣰⠾⢓⡵⣺⢺⣼⡫⢪⡱⣉⠷⢗⡀⠤⠆⡻⣛⠿⣻⣿⢶⣊⡄⠀⠀⠀⠀⠄⢀⠀⠉⠿⣿⡿⣿⣛⡁⢍⣀⡌
# ⠀⠀⠀⠀⠀⠀⣠⣛⢓⠗⠀⠀⠠⣈⠉⢀⢬⣵⡿⢋⣴⣞⣵⣼⣭⢁⠕⢿⢋⣞⢟⣕⡩⣔⠃⠀⠀⡀⣛⣤⢿⣷⢻⣿⣿⣽⣮⡙⠆⠀⠤⢓⡙⣆⠀⠀⠘⠙⢯⣛⣶⠋⠁
# ⠀⠀⠀⠀⠀⢠⢋⢿⣼⣶⣶⣤⠒⢉⠠⣪⣮⠟⣡⡾⠹⡿⣿⣿⠝⢊⣐⢺⡜⣫⡞⢭⡕⠋⣐⠒⠀⠡⠏⠉⠘⠛⠚⡻⣯⠋⠛⢅⠐⢄⠀⣸⡃⠛⠀⡀⡀⠀⠈⠙⡟⠀⠀
# ⠀⠀⠀⠀⣠⢫⠎⠙⠿⣿⠛⡡⠔⠕⣴⡿⡁⡮⡷⡶⢟⣿⢎⡵⡠⢞⠱⢈⣼⠁⠄⠇⡄⣢⠒⠀⡎⠀⡇⠒⠐⠐⠐⢚⠐⢷⣔⢖⢊⡈⠒⠗⠠⠘⠈⡁⢈⣠⣤⠾⠀⠀⠀
# ⠀⠀⠀⣰⢳⠏⢀⢔⢤⠶⠪⣠⣭⣾⣿⠗⢘⣷⣼⠛⠛⢛⡝⣜⢑⣤⣾⠿⣿⣿⢽⣿⠿⠶⢴⣯⣄⡄⣇⣀⣀⡀⠄⠠⠆⣀⡨⢽⣵⣕⣄⣀⣰⣥⣶⣾⡿⠟⠉⠀⠀⠀⠀
# ⠀⠀⡰⣱⢋⠴⣩⠔⠻⣴⡾⢷⣿⡿⠃⠰⢿⣿⣿⣿⣶⣬⣧⣼⡿⠛⠁⡢⠒⠈⢈⡉⠿⠚⢼⣿⣿⣿⡆⠋⠉⠢⠀⢀⣀⣡⣴⡶⣿⣿⣿⠿⠻⠛⠋⠁⠀⠀⠀⠀⠀⠀⠀
# ⠀⡼⣳⣯⣱⣪⣲⣫⠻⣯⢟⠽⠋⠀⠀⠀⠀⠈⠙⠻⢻⡳⡩⢇⢀⠸⢢⣤⣴⣁⡀⠊⡀⠠⠂⢉⣫⡭⣁⣀⣠⣴⣶⢿⣿⣿⣿⡿⠞⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⣼⣟⡿⣿⣿⢝⣫⣶⡿⠣⠉⠀⠀⠀⠀⠀⠀⠀⣠⣖⠕⡩⢂⢕⠡⠒⠸⣿⣿⣿⡿⢂⢀⠀⣜⡿⣵⣶⣾⣿⡿⠯⠟⠋⠉⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⢸⢿⣷⣷⣷⣿⠛⠋⠁⠀⠀⠀⠀⠀⠀⢀⣴⡺⠟⣑⣿⡺⢑⡴⠂⠊⠀⢀⡁⣍⣢⣼⣺⣽⣶⡿⠿⠏⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠈⠙⠻⠝⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⡿⡋⢰⠕⠋⡿⠉⢁⡈⢕⣲⣼⣒⣯⣷⣿⠿⠟⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣴⣿⣷⣧⡎⣠⡤⠥⣰⢬⣵⣮⣽⣿⡿⠟⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣷⣮⣟⡯⣓⣦⣿⣮⣿⣿⣿⠿⠛⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠛⠿⣿⣿⣿⣿⡿⠟⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
# ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠛⠉⠀⠀
