#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    WrenchStamped,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from velocity_controller_lqr.velocity_controller_lqr_lib import (
    GuidanceValues,
    LQRController,
    LQRParameters,
    State,
)
from vortex_msgs.msg import LOSGuidance


class LinearQuadraticRegulator(LifecycleNode):
    def __init__(self):
        # ----------------------- DEFINE RELIABILITY ------------------------
        super().__init__("velocity_controller_lqr_node")
        self.best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        # ---------------- CALLBACK VARIABLES INITIALIZATION ----------------
        self.coriolis_matrix = np.zeros((3, 3))
        self.inertia_matrix = np.zeros((3, 3))
        self.states = State()
        self.guidance_values = GuidanceValues()
        self.lqr_params = LQRParameters()
        self.dt = None
        # --------------------------- SUBSCRIBERS --------------------------
        self.pose_subscriber = None
        self.twist_subscriber = None
        self.operation_mode_subscriber = None
        self.killswitch_subscriber = None
        self.guidance_subscriber = None
        # ------------------------ CONTROLLER MODES ------------------------
        self.killswitch = None
        self.operation_mode = None
        # --------------------------- PUBLISHERS ---------------------------
        self.publisherLQR = None
        # ----------------------------- TIMERS -----------------------------
        self.control_timer = None
        # ------------------ ROS2 PARAMETERS AND CONTROLLER ------------------
        self.get_and_reshape_inertia_matrix()
        self.controller = LQRController(self.lqr_params, self.inertia_matrix)

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.declare_params()
        self.get_logger().info("1")
        self.get_params()
        self.get_logger().info("2")
        # -------------------------- GET ALL TOPICS -------------------------
        (
            pose_topic,
            twist_topic,
            guidance_topic,
            thrust_topic,
            software_operation_topic,
            killswitch_topic,
        ) = self.get_topics()

        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self.pose_callback,
            qos_profile=self.best_effort_qos,
        )

        self.twist_subscriber = self.create_subscription(
            TwistWithCovarianceStamped,
            twist_topic,
            self.twist_callback,
            qos_profile=self.best_effort_qos,
        )

        self.operation_mode_subscriber = self.create_subscription(
            String,
            software_operation_topic,
            self.operation_callback,
            qos_profile=self.reliable_qos,
        )
        self.killswitch_subscriber = self.create_subscription(
            Bool,
            killswitch_topic,
            self.killswitch_callback,
            qos_profile=self.reliable_qos,
        )

        self.guidance_subscriber = self.create_subscription(
            LOSGuidance,
            guidance_topic,
            self.guidance_callback,
            qos_profile=self.best_effort_qos,
        )

        # ---------------------------- PUBLISHERS ----------------------------
        self.publisherLQR = self.create_lifecycle_publisher(
            WrenchStamped, thrust_topic, self.reliable_qos
        )

        # ------------------------------ TIMERS ------------------------------
        dt = self.dt
        self.control_timer = self.create_timer(dt, self.control_loop)
        self.control_timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.control_timer.reset()
        self.controller.reset_controller()
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.control_timer.cancel()
        self.controller.reset_controller()
        return super().on_deactivate(previous_state)

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisherLQR)
        self.destroy_timer(self.control_timer)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisherLQR)
        self.destroy_timer(self.control_timer)
        return TransitionCallbackReturn.SUCCESS

    def get_topics(self) -> None:
        """Get the topics from the parameter server.

        Returns:
        pose_topic: str: The topic for accessing the pose data from the parameter file
        twist_topic: str: The topic for accessing the twist data from the parameter file
        guidance_topic: str: The topic for accessing the guidance data the parameter file
        thrust_topic: str: The topic for accessing the thrust data from the parameter file
        software_operation_mode_topic: str: The topic for accessing the operation mode from the parameter file
        killswitch_topic: str: The topic for accessing the killswitch bool from the parameter file

        """
        self.declare_parameter("topics.pose", "_")
        self.declare_parameter("topics.twist", "_")
        self.declare_parameter("topics.guidance.los", "_")
        self.declare_parameter("topics.wrench_input", "_")
        self.declare_parameter("topics.operation_mode", "_")
        self.declare_parameter("topics.killswitch", "_")

        pose_topic = self.get_parameter("topics.pose").value
        twist_topic = self.get_parameter("topics.twist").value
        guidance_topic = self.get_parameter("topics.guidance.los").value
        thrust_topic = self.get_parameter("topics.wrench_input").value
        operation_topic = self.get_parameter("topics.operation_mode").value
        killswitch_topic = self.get_parameter("topics.killswitch").value

        return (
            pose_topic,
            twist_topic,
            guidance_topic,
            thrust_topic,
            operation_topic,
            killswitch_topic,
        )

    def declare_params(self) -> None:
        """Declares parameters that are to be used from the configuration file."""
        self.get_logger().info("cool")

        self.declare_parameter("LQR_params.q_surge", 0.0)
        self.declare_parameter("LQR_params.q_pitch", 0.0)
        self.declare_parameter("LQR_params.q_yaw", 0.0)

        self.get_logger().info("yaw")

        self.declare_parameter("LQR_params.r_surge", 0.0)
        self.declare_parameter("LQR_params.r_pitch", 0.0)
        self.declare_parameter("LQR_params.r_yaw", 0.0)

        self.get_logger().info("shaw")

        self.declare_parameter("LQR_params.i_surge", 0.0)
        self.declare_parameter("LQR_params.i_pitch", 0.0)
        self.declare_parameter("LQR_params.i_yaw", 0.0)

        self.get_logger().info("garhamat")

        self.declare_parameter("LQR_params.i_weight", 0.0)

        self.declare_parameter("dt", 0.0)
        self.declare_parameter("propulsion.thrusters.max", 0.0)

    def get_params(self) -> None:
        """Gets the declared parameters from the configuration file."""
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
        self.lqr_params.max_force = self.get_parameter("propulsion.thrusters.max").value

        self.dt = self.get_parameter("dt").value

    def get_and_reshape_inertia_matrix(self) -> None:
        """Gets the inertia matrix from config and reshapes it to proper np array."""
        self.declare_parameter("inertia_matrix")
        self.inertia_matrix = self.get_parameter("inertia_matrix").value
        inertia_matrix_reshaped = np.array(self.inertia_matrix).reshape((3, 3))

        self.inertia_matrix = inertia_matrix_reshaped

    # ------------------------- CALLBACK FUNCTIONS ---------------------------

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Callback function for the pose data from sensors.

        Parameters: msg: PoseWithCovarianceStamped The pose data from the DVL.

        """
        _, self.states.pitch, self.states.yaw = LQRController.quaternion_to_euler_angle(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

    def operation_callback(self, msg: String) -> None:
        """Callback function for the operation mode data.

        Parameters: String: msg: The operation mode data from the AUV.

        """
        self.operation_mode = msg.data

    def twist_callback(self, msg: TwistWithCovarianceStamped) -> None:
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

    def guidance_callback(self, msg: LOSGuidance) -> None:
        """Callback function for the guidance data.

        Parameters: LOSGuidance: msg: The guidance data from the LOS guidance system.

        """
        self.guidance_values.surge = msg.surge
        self.guidance_values.pitch = msg.pitch
        self.guidance_values.yaw = msg.yaw

    def killswitch_callback(self, msg: Bool) -> None:
        """Callback function for the killswitch data.

        Parameters: String: msg: The killswitch data from the AUV.

        """
        if msg.data == True:
            self.controller.reset_controller()
            self.killswitch = True
        else:
            self.killswitch = False

    # ---------------------------------------------------------------PUBLISHER FUNCTIONS-------------------------------------------------------------

    def control_loop(self) -> None:
        """The control loop that calculates the input for the LQR controller."""
        if self.killswitch == True or self.operation_mode != "autonomous mode":
            self.controller.reset_controller()
            return

        msg = WrenchStamped()

        u = self.controller.calculate_lqr_u(
            self.coriolis_matrix, self.states, self.guidance_values
        )

        msg.wrench.force.x = float(u[0])
        msg.wrench.torque.y = float(u[1])
        msg.wrench.torque.z = float(u[2])

        self.publisherLQR.publish(msg)


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
