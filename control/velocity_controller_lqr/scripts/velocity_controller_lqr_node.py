#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    Wrench,
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
            depth=10,
        )

        # ---------------- CALLBACK VARIABLES INITIALIZATION ----------------
        self.coriolis_matrix = np.zeros((3, 3))
        self.inertia_matrix = np.zeros((3,3))
        self.states = State()
        self.guidance_values = GuidanceValues()
        self.lqr_params = LQRParameters()
        # ------------------------ INITIALIZE TO NONE -----------------------
        self.pose_subscriber = None
        self.twist_subscriber = None
        self.operationmode_subscriber = None
        self.killswitch_subscriber = None
        self.guidance_subscriber = None
        self.publisherLQR = None
        self.control_timer = None

        # ------------------ ROS2 PARAMETERS AND CONTROLLER ------------------
        self.get_and_reshape_inertia_matrix()
        self.controller = LQRController(self.lqr_params, self.inertia_matrix)

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        
        self.get_parameters()
        # -------------------------- GET ALL TOPICS -------------------------
        (
            pose_topic,
            twist_topic,
            guidance_topic,
            thrust_topic,
            softwareoperation_topic,
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

        self.operationmode_subscriber = self.create_subscription(
            String,
            softwareoperation_topic,
            self.operation_callback,
            qos_profile=self.best_effort_qos,
        )
        self.killswitch_subscriber = self.create_subscription(
            Bool,
            killswitch_topic,
            self.killswitch_callback,
            qos_profile=self.best_effort_qos,
        )

        self.guidance_subscriber = self.create_subscription(
            LOSGuidance,
            guidance_topic,
            self.guidance_callback,
            qos_profile=self.reliable_qos,
        )

        # ---------------------------- PUBLISHERS ----------------------------
        self.publisherLQR = self.create_lifecycle_publisher(
            Wrench, thrust_topic, self.reliable_qos
        )

        # ------------------------------ TIMERS ------------------------------
        dt = self.lqr_params.dt
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
        return super().on_activate(previous_state)

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
        odom_topic: str: The topic for accessing the odometry data from the parameter file
        twist_topic: str: The topic for accessing the twist data from the parameter file
        pose_topic: str: The topic for accessing the pose data from the parameter file
        guidance_topic: str: The topic for accessing the guidance data the parameter file
        thrust_topic: str: The topic for accessing the thrust data from the parameter file
        """
        self.declare_parameter("topics.pose_topic", "/dvl/pose")
        self.declare_parameter("topics.twist_topic", "/dvl/twist")
        self.declare_parameter("topics.guidance_topic", "/guidance/los")
        self.declare_parameter("topics.thrust_topic", "/thrust/wrench_input")
        self.declare_parameter(
            "topics.softwareoperation_topic", "/softwareOperationMode"
        )
        self.declare_parameter("topics.killswitch_topic", "/softwareKillSwitch")

        pose_topic = self.get_parameter("topics.pose_topic").value
        twist_topic = self.get_parameter("topics.twist_topic").value
        guidance_topic = self.get_parameter("topics.guidance_topic").value
        thrust_topic = self.get_parameter("topics.thrust_topic").value
        softwareoperation_topic = self.get_parameter(
            "topics.softwareoperation_topic"
        ).value
        killswitch_topic = self.get_parameter("topics.killswitch_topic").value

        return (
            pose_topic,
            twist_topic,
            guidance_topic,
            thrust_topic,
            softwareoperation_topic,
            killswitch_topic,
        )

    def get_parameters(self):
        """Updates the LQR_params in the LQR_parameters Dataclass."""
        
        self.declare_parameter("LQR_params.q_surge")
        self.declare_parameter("LQR_params.q_pitch")
        self.declare_parameter("LQR_params.q_yaw")

        self.declare_parameter("LQR_params.r_surge")
        self.declare_parameter("LQR_params.r_pitch")
        self.declare_parameter("LQR_params.r_yaw")

        self.declare_parameter("LQR_params.i_surge")
        self.declare_parameter("LQR_params.i_pitch")
        self.declare_parameter("LQR_params.i_yaw")

        self.declare_parameter("LQR_params.i_weight")

        self.declare_parameter("LQR_params.dt")

        self.declare_parameter("max_force")

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

        self.lqr_params.dt = self.get_parameter("LQR_params.dt").value


    def get_and_reshape_inertia_matrix(self):
        """Gets the inertia matrix from config and reshapes it to proper np array"""
        self.declare_parameter("inertia_matrix")
        self.inertia_matrix = self.get_parameter("inertia_matrix").value
        inertia_matrix_reshaped = np.array(self.inertia_matrix).reshape((3, 3))
        
        self.inertia_matrix = inertia_matrix_reshaped


    # ------------------------- CALLBACK FUNCTIONS ---------------------------

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback function for the pose data from sensors.

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
        if msg.data == True:
            self.controller.reset_controller()
            self.controller.killswitch = True
        else:
            self.controller.killswitch = False

    # ---------------------------------------------------------------PUBLISHER FUNCTIONS-------------------------------------------------------------

    def control_loop(self):
        """The control loop that calculates the input for the LQR controller."""
        if (
            self.controller.killswitch == False
            and self.controller.operation_mode == "autonomous mode"
        ):
            self.publisherLQR.publish(msg)

        else:
            self.controller.reset_controller()
            
        msg = Wrench()
        

        u = self.controller.calculate_lqr_u(
            self.coriolis_matrix, self.states, self.guidance_values
        )
        msg.force.x = float(u[0])
        msg.torque.y = float(u[1])
        msg.torque.z = float(u[2])
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
