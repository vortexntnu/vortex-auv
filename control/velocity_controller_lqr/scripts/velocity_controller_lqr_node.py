#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    WrenchStamped,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, Parameter
from std_msgs.msg import Bool, String
from velocity_controller_lqr.velocity_controller_lqr_lib import (
    LQRController,
    LQRParameters,
)
from vortex_msgs.msg import LOSGuidance, OperationMode
from vortex_utils.python_utils import State
from vortex_utils_ros.qos_profiles import reliable_profile, sensor_data_profile
from vortex_utils_ros.ros_converter import pose_from_ros, twist_from_ros


class LinearQuadraticRegulator(Node):
    def __init__(self):
        super().__init__("velocity_controller_lqr_node")

        self.killswitch_on = True
        self.operation_mode = OperationMode.MANUAL

        self.get_topics()

        # ---------------------------- SUBSCRIBERS ---------------------------

        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            qos_profile=sensor_data_profile(1),
        )

        self.twist_subscriber = self.create_subscription(
            TwistWithCovarianceStamped,
            self.twist_topic,
            self.twist_callback,
            qos_profile=sensor_data_profile(1),
        )

        self.operationmode_subscriber = self.create_subscription(
            OperationMode,
            self.operation_mode_topic,
            self.operation_callback,
            qos_profile=reliable_profile(2),
        )
        self.killswitch_subscriber = self.create_subscription(
            Bool,
            self.killswitch_topic,
            self.killswitch_callback,
            qos_profile=reliable_profile(2),
        )

        self.guidance_subscriber = self.create_subscription(
            LOSGuidance,
            self.los_topic,
            self.guidance_callback,
            qos_profile=sensor_data_profile(1),
        )

        # ---------------------------- PUBLISHERS ----------------------------
        self.publisherLQR = self.create_publisher(
            WrenchStamped, self.wrench_input_topic, sensor_data_profile(1)
        )

        # ------------------------------ TIMERS ------------------------------
        dt = (
            self.declare_parameter("dt", Parameter.Type.DOUBLE)
            .get_parameter_value()
            .double_value
        )
        self.control_timer = self.create_timer(dt, self.control_loop)

        # ------------------ ROS2 PARAMETERS AND CONTROLLER ------------------
        self.lqr_params = LQRParameters()
        self.fill_lqr_params()
        inertia_matrix_flat = (
            self.declare_parameter("inertia_matrix", Parameter.Type.DOUBLE_ARRAY)
            .get_parameter_value()
            .double_array_value
        )
        inertia_matrix = np.array(inertia_matrix_flat).reshape((3, 3))
        self.controller = LQRController(self.lqr_params, inertia_matrix)

        # ---------------- CALLBACK VARIABLES INITIALIZATION -----------------
        self.state = State()
        self.guidance_values = State()

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
                self.declare_parameter(
                    "topics.guidance." + topic, Parameter.Type.STRING
                )
                setattr(
                    self,
                    topic + "_topic",
                    self.get_parameter("topics.guidance." + topic).value,
                )
                continue
            self.declare_parameter("topics." + topic, Parameter.Type.STRING)
            setattr(
                self,
                topic + "_topic",
                self.get_parameter("topics." + topic).value,
            )

    def fill_lqr_params(self):
        """Updates the LQR_params in the LQR_parameters Dataclass, and gets the inertia matrix from config.

        Returns:
        inertia_matrix: np.array: The inertia matrix of the AUV
        """
        self.declare_parameter("LQR_params.q_surge", Parameter.Type.INTEGER)
        self.declare_parameter("LQR_params.q_pitch", Parameter.Type.INTEGER)
        self.declare_parameter("LQR_params.q_yaw", Parameter.Type.INTEGER)

        self.declare_parameter("LQR_params.r_surge", Parameter.Type.DOUBLE)
        self.declare_parameter("LQR_params.r_pitch", Parameter.Type.DOUBLE)
        self.declare_parameter("LQR_params.r_yaw", Parameter.Type.DOUBLE)

        self.declare_parameter("LQR_params.i_surge", Parameter.Type.DOUBLE)
        self.declare_parameter("LQR_params.i_pitch", Parameter.Type.DOUBLE)
        self.declare_parameter("LQR_params.i_yaw", Parameter.Type.DOUBLE)

        self.declare_parameter("LQR_params.i_weight", Parameter.Type.DOUBLE)

        self.declare_parameter("max_force", Parameter.Type.DOUBLE)

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

    # ---------------------------------------------------------------CALLBACK FUNCTIONS---------------------------------------------------------------

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback function for the pose data from DVL.

        Parameters: msg: PoseWithCovarianceStamped The pose data from the DVL.

        """
        self.state.pose = pose_from_ros(msg.pose.pose)

    def operation_callback(self, msg: OperationMode):
        """Callback function for the operation mode data.

        Parameters: String: msg: The operation mode data from the AUV.

        """
        self.operation_mode = msg.operation_mode
        self.get_logger().info(f"Changed operation mode to {self.operation_mode}")

    def twist_callback(self, msg: TwistWithCovarianceStamped):
        """Callback function for the Twist data from DVL.

        Parameters: msg: TwistWithCovarianceStamped The twist data from the DVL.

        """
        self.state.twist = twist_from_ros(msg.twist.twist)

    def guidance_callback(self, msg: LOSGuidance):
        """Callback function for the guidance data.

        Parameters: LOSGuidance: msg: The guidance data from the LOS guidance system.

        """
        self.guidance_values.twist.linear_x = msg.surge
        self.guidance_values.pose.pitch = msg.pitch
        self.guidance_values.pose.yaw = msg.yaw

    def killswitch_callback(self, msg: Bool):
        """Callback function for the killswitch data.

        Parameters: String: msg: The killswitch data from the AUV.

        """
        self.killswitch_on = msg.data
        self.get_logger().info(f"Killswitch {'on' if self.killswitch_on else 'off'}")
        if self.killswitch_on:
            self.controller.reset_controller()

    # ---------------------------------------------------------------PUBLISHER FUNCTIONS-------------------------------------------------------------

    def control_loop(self):
        """The control loop that calculates the input for the LQR controller."""
        msg = WrenchStamped()

        u = self.controller.calculate_lqr_u(
            state=self.state, guidance_values=self.guidance_values
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.wrench.force.x = float(u[0])
        msg.wrench.torque.y = float(u[1])
        msg.wrench.torque.z = float(u[2])

        if self.killswitch_on == False and (
            self.operation_mode == OperationMode.AUTONOMOUS
            or self.operation_mode == OperationMode.REFERENCE
        ):
            self.publisherLQR.publish(msg)

        else:
            self.controller.reset_controller()


# ----------------------------------------------------------------------MAIN FUNCTION----------------------------------------------------------------

start_message = """
  _     ___  ____     ____            _             _ _
 | |   / _ \|  _ \   / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __
 | |  | | | | |_) | | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|
 | |__| |_| |  _ <  | |__| (_) | | | | |_| | | (_) | | |  __/ |
 |_____\__\_\_| \_\  \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|

                                                                  """


def main(args=None):
    rclpy.init(args=args)
    lqr_node = LinearQuadraticRegulator()
    executor = MultiThreadedExecutor()
    executor.add_node(lqr_node)
    lqr_node.get_logger().info(start_message)
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
