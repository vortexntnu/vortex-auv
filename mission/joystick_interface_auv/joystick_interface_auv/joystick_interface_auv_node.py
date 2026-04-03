#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, WrenchStamped
from rclpy.node import Node, Parameter
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool
from vortex_msgs.msg import OperationMode, ReferenceFilter, ReferenceFilterQuat
from vortex_msgs.srv import GetOperationMode, SetOperationMode, ToggleKillswitch
from vortex_utils.python_utils import PoseData, euler_to_quat
from vortex_utils_ros.ros_converter import pose_from_ros
from vortex_utils_ros.qos_profiles import (
    reliable_profile,
    sensor_data_profile,
)

from joystick_interface_auv.joystick_utils import (
    Wired,
    WirelessXboxSeriesX,
    modes,
)

start_message = r"""
      _                 _   _      _      ___       _             __
     | | ___  _   _ ___| |_(_) ___| | __ |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
  _  | |/ _ \| | | / __| __| |/ __| |/ /  | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
 | |_| | (_) | |_| \__ \ |_| | (__|   <   | || | | | ||  __/ |  |  _| (_| | (_|  __/
  \___/ \___/ \__, |___/\__|_|\___|_|\_\ |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
              |___/
"""


class JoystickInterface(Node):
    def __init__(self):
        super().__init__('joystick_interface_auv')

        self.get_parameters()
        self.init_movement()
        self.set_publishers_and_subscribers()
        self.set_services()

        self._current_state = PoseData()
        self._desired_state = PoseData()

        self._mode = OperationMode.MANUAL
        self._killswitch = True

        try:
            request = GetOperationMode.Request()
            future = self.get_operation_mode_client.call_async(request)
            future.add_done_callback(self.handle_initial_operation_mode_response)
        except Exception as e:
            self.get_logger().error(f"Failed to call GetOperationMode service: {e}")
            self._killswitch = True

        self._joystick_axes_map = []
        self._joystick_buttons_map = []
        self._last_button_press_time = 0

        self.get_logger().info(start_message)

    def get_parameters(self):
        """Method to get the parameters from the config file."""
        self.declare_parameter('drone', 'orca')
        self._drone = self.get_parameter('drone').value

        gain_params = [
            'joystick_surge_gain',
            'joystick_sway_gain',
            'joystick_heave_gain',
            'joystick_roll_gain',
            'joystick_pitch_gain',
            'joystick_yaw_gain',
            'guidance_surge_gain',
            'guidance_sway_gain',
            'guidance_heave_gain',
            'guidance_roll_gain',
            'guidance_pitch_gain',
            'guidance_yaw_gain',
            'debounce_duration',
        ]

        for param in gain_params:
            self.declare_parameter(param, Parameter.Type.DOUBLE)
            # Get the values and set them as attributes of the class
            setattr(self, '_' + param, self.get_parameter(param).value)

        topic_params = [
            'pose',
            'joy',
            'wrench_input',
            'killswitch',
            'operation_mode',
            'gripper_servos',
        ]

        for param in topic_params:
            self.declare_parameter(f'topics.{param}', Parameter.Type.STRING)
            setattr(
                self,
                param + '_topic',
                self.get_parameter(f'topics.{param}').value,
            )

        service_params = [
            'set_operation_mode',
            'toggle_killswitch',
            'get_operation_mode',
        ]

        for param in service_params:
            self.declare_parameter(f'services.{param}', Parameter.Type.STRING)
            setattr(
                self,
                param + '_service',
                self.get_parameter(f'services.{param}').value,
            )

        self.declare_parameter('orientation_mode', 'euler')
        self._orientation_mode = self.get_parameter('orientation_mode').value
        if self._orientation_mode not in ('euler', 'quat'):
            self.get_logger().warn(
                f"Unknown orientation_mode '{self._orientation_mode}', defaulting to 'euler'"
            )
            self._orientation_mode = 'euler'

        self.declare_parameter('topics.guidance.dp_rpy', Parameter.Type.STRING)
        self.declare_parameter('topics.guidance.dp_quat', Parameter.Type.STRING)
        if self._orientation_mode == 'quat':
            self.guidance_topic = self.get_parameter('topics.guidance.dp_quat').value
        else:
            self.guidance_topic = self.get_parameter('topics.guidance.dp_rpy').value

    def init_movement(self):
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def set_publishers_and_subscribers(self):
        best_effort_qos = sensor_data_profile(1)
        reliable_qos = reliable_profile(1)

        self._joy_subscriber = self.create_subscription(
            Joy, self.joy_topic, self.joystick_cb, qos_profile=best_effort_qos
        )
        self._pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_cb,
            qos_profile=best_effort_qos,
        )
        self._mode_subscriber = self.create_subscription(
            OperationMode,
            self.operation_mode_topic,
            self.operation_mode_cb,
            qos_profile=reliable_qos,
        )
        self._killswitch_subscriber = self.create_subscription(
            Bool,
            self.killswitch_topic,
            self.killswitch_cb,
            qos_profile=reliable_qos,
        )
        self._wrench_publisher = self.create_publisher(
            WrenchStamped, self.wrench_input_topic, qos_profile=best_effort_qos
        )
        if self._orientation_mode == 'quat':
            self._ref_publisher = self.create_publisher(
                ReferenceFilterQuat, self.guidance_topic, qos_profile=best_effort_qos
            )
        else:
            self._ref_publisher = self.create_publisher(
                ReferenceFilter, self.guidance_topic, qos_profile=best_effort_qos
            )

        self._gripper_publisher = self.create_publisher(
            JointState, self.gripper_servos_topic, qos_profile=best_effort_qos
        )

    def set_services(self):
        self.operation_mode_client = self.create_client(
            SetOperationMode, self.set_operation_mode_service
        )

        while not self.operation_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Operation Mode service not available, waiting...')

        self.toggle_killswitch_client = self.create_client(
            ToggleKillswitch, self.toggle_killswitch_service
        )

        while not self.toggle_killswitch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Toggle Killswitch service not available, waiting...'
            )

        self.get_operation_mode_client = self.create_client(
            GetOperationMode, self.get_operation_mode_service
        )

        while not self.get_operation_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Get Operation Mode service not available, waiting...'
            )

    def handle_initial_operation_mode_response(self, future):
        """Handle the response from the GetOperationMode service call to initialize the operation mode.

        Parameters: future: The future object containing the response from the service call.

        """
        try:
            response = future.result()
            self._mode = response.current_operation_mode
            self._killswitch = response.killswitch_status
            self.get_logger().info(
                f"Initial operation mode: {self._mode} | Initial killswitch status: {'on' if self._killswitch else 'off'}"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """Callback function for the pose subscriber. Updates the current state of the AUV."""
        self._current_state = pose_from_ros(msg.pose.pose)

    def operation_mode_cb(self, msg: OperationMode):
        self._mode = msg.operation_mode

    def killswitch_cb(self, msg: Bool):
        self._killswitch = msg.data

    def create_reference_message(self) -> ReferenceFilter:
        """Creates a reference message with the desired state values."""
        reference_msg = ReferenceFilter()
        reference_msg.header.stamp = self.get_clock().now().to_msg()
        # reference_msg.header.frame_id = "odom"
        reference_msg.header.frame_id = "base_link"
        reference_msg.x = self._desired_state.x
        reference_msg.y = self._desired_state.y
        reference_msg.z = self._desired_state.z
        reference_msg.roll = self._desired_state.roll
        reference_msg.pitch = self._desired_state.pitch
        reference_msg.yaw = self._desired_state.yaw
        return reference_msg

    def create_reference_quat_message(self) -> ReferenceFilterQuat:
        """Creates a reference message with quaternion orientation from the desired state."""
        q = euler_to_quat(
            self._desired_state.roll,
            self._desired_state.pitch,
            self._desired_state.yaw,
        )
        reference_msg = ReferenceFilterQuat()
        reference_msg.header.stamp = self.get_clock().now().to_msg()
        reference_msg.header.frame_id = "base_link"
        reference_msg.x = self._desired_state.x
        reference_msg.y = self._desired_state.y
        reference_msg.z = self._desired_state.z
        reference_msg.qx = float(q[0])
        reference_msg.qy = float(q[1])
        reference_msg.qz = float(q[2])
        reference_msg.qw = float(q[3])
        return reference_msg

    def create_wrench_message(self) -> WrenchStamped:
        """Creates a 3D wrench message with the given x, y, heave, roll, pitch, and yaw values.

        Returns:
        Wrench: A 3D wrench message with the given values.
        """
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = "base_link"
        wrench_msg.wrench.force.x = self.surge
        wrench_msg.wrench.force.y = self.sway
        wrench_msg.wrench.force.z = self.heave
        wrench_msg.wrench.torque.x = self.roll
        wrench_msg.wrench.torque.y = self.pitch
        wrench_msg.wrench.torque.z = self.yaw
        return wrench_msg

    def transition_to_xbox_mode(self):
        """Turns off the controller and signals that the operational mode has switched to Xbox mode."""
        self.get_logger().info("Turning on Xbox controller")
        request = SetOperationMode.Request()
        request.requested_operation_mode.operation_mode = OperationMode.MANUAL
        future = self.operation_mode_client.call_async(request)
        future.add_done_callback(self.operation_mode_response_callback)
        self.get_logger().info("XBOX mode")

    def _create_reference_msg(self):
        """Returns the appropriate reference message based on orientation_mode."""
        if self._orientation_mode == 'quat':
            return self.create_reference_quat_message()
        return self.create_reference_message()

    def transition_to_reference_mode(self):
        """Publishes a pose message and signals that the operational mode has switched to Reference mode."""
        self._desired_state = PoseData(
            x=self._current_state.x,
            y=self._current_state.y,
            z=self._current_state.z,
            roll=self._current_state.roll,
            pitch=self._current_state.pitch,
            yaw=self._current_state.yaw,
        )
        reference_msg = self._create_reference_msg()
        # Still autonomous mode, but now the reference is being controlled by the joystick

        request = SetOperationMode.Request()
        request.requested_operation_mode.operation_mode = OperationMode.REFERENCE
        future = self.operation_mode_client.call_async(request)
        future.add_done_callback(self.operation_mode_response_callback)
        self.get_logger().info("Reference mode")
        self._ref_publisher.publish(reference_msg)

    def transition_to_autonomous_mode(self):
        """Publishes a zero force wrench message and signals that the system is turning on autonomous mode."""
        empty_wrench_msg = WrenchStamped()
        empty_wrench_msg.header.stamp = self.get_clock().now().to_msg()
        empty_wrench_msg.header.frame_id = "base_link"
        self._wrench_publisher.publish(empty_wrench_msg)

        request = SetOperationMode.Request()
        request.requested_operation_mode.operation_mode = OperationMode.AUTONOMOUS
        future = self.operation_mode_client.call_async(request)
        future.add_done_callback(self.operation_mode_response_callback)
        self.get_logger().info("Autonomous mode")

    def operation_mode_response_callback(self, future):
        """Callback function for the operation mode service response."""
        response = future.result()
        if response.current_operation_mode.operation_mode in modes:
            self.get_logger().info(
                f"Operation mode set to: {modes[response.current_operation_mode.operation_mode]} : Killswitch {response.killswitch_status}"
            )
        else:
            self.get_logger().error("Failed to set operation mode.")

    def check_number_of_buttons(self, msg: Joy):
        """Checks if the controller is wireless (has 16 buttons) or wired and sets the joystick button and axis maps accordingly.

        Args:
            msg: A ROS message containing the joy input data.
        """
        if len(msg.buttons) == 16:
            self._joystick_buttons_map = WirelessXboxSeriesX.joystick_buttons_map
            self._joystick_axes_map = WirelessXboxSeriesX.joystick_axes_map
        else:
            self._joystick_buttons_map = Wired.joystick_buttons_map
            self._joystick_axes_map = Wired.joystick_axes_map

    def populate_buttons_dictionary(self, msg: Joy) -> dict:
        """Populates a dictionary with button JoyStates from the joystick message.

        Args:
            msg: A ROS message containing the joy input data.

        Returns:
            A dictionary with button names as keys and their JoyStates as values.
        """
        buttons = {}
        for i, button_name in enumerate(self._joystick_buttons_map):
            if i < len(msg.buttons):
                buttons[button_name] = msg.buttons[i]
            else:
                # Assuming default value if button is not present
                buttons[button_name] = 0
        return buttons

    def populate_axes_dictionary(self, msg: Joy) -> dict:
        """Populates a dictionary with axis values from the joystick message.

        Args:
            msg: A ROS message containing the joy input data.

        Returns:
            A dictionary with axis names as keys and their values as values.
        """
        axes = {}
        for i, axis_name in enumerate(self._joystick_axes_map):
            if i < len(msg.axes):
                axes[axis_name] = msg.axes[i]
            else:
                # Assuming default value if axis is not present
                axes[axis_name] = 0.0
        return axes

    def calculate_movement(self, axes: dict, buttons: dict):
        """Calculates the 6 DOF movement vector based on joystick input."""
        left_trigger = axes.get("LT", 0.0)
        right_trigger = axes.get("RT", 0.0)
        left_shoulder = buttons.get("LB", 0)
        right_shoulder = buttons.get("RB", 0)

        self.surge = (
            axes.get("vertical_axis_left_stick", 0.0) * self._joystick_surge_gain
        )
        self.sway = (
            -axes.get("horizontal_axis_left_stick", 0.0) * self._joystick_sway_gain
        )
        self.heave = -(left_trigger - right_trigger) * self._joystick_heave_gain
        self.roll = (right_shoulder - left_shoulder) * self._joystick_roll_gain
        self.pitch = (
            -axes.get("vertical_axis_right_stick", 0.0) * self._joystick_pitch_gain
        )
        self.yaw = (
            -axes.get("horizontal_axis_right_stick", 0.0) * self._joystick_yaw_gain
        )

    def handle_killswitch_button(self) -> None:
        """Handles the software killswitch button press.

        This function performs the following actions based on the current state:
        1. If the current state is KILLSWITCH, it signals that the killswitch is not blocking,
            transitions to Xbox mode, and returns.
        2. Otherwise, it logs a message indicating that the software killswitch is active,
            signals that the killswitch is blocking, publishes a zero wrench message to stop
            the AUV, and sets the state to KILLSWITCH.

        The function ensures that the AUV stops moving when the killswitch is activated
        and allows it to resume operation when the killswitch is deactivated.
        """
        request = ToggleKillswitch.Request()
        future = self.toggle_killswitch_client.call_async(request)
        future.add_done_callback(self.operation_mode_response_callback)

    def update_reference(self):
        """Updates the current pose of the AUV based on joystick inputs.

        The position and orientation (roll, pitch, yaw) are updated
        using the current joystick inputs scaled by their respective parameters.
        The linear velocities (surge, sway, heave) are transformed from the
        body frame to the world frame using the current orientation.
        """
        surge_vector = self.surge * self._guidance_surge_gain
        sway_vector = self.sway * self._guidance_sway_gain
        heave_vector = -self.heave * self._guidance_heave_gain

        body_frame_vector = np.array([surge_vector, sway_vector, heave_vector])
        
        rotation_matrix = self._desired_state.as_rotation_matrix()
        world_frame_vector = rotation_matrix @ body_frame_vector
        
        self._desired_state.x += world_frame_vector[0]
        self._desired_state.y += world_frame_vector[1]
        self._desired_state.z += world_frame_vector[2]

        self._desired_state.roll += self.roll * self._guidance_roll_gain
        self._desired_state.pitch += self.pitch * self._guidance_pitch_gain
        self._desired_state.yaw += self.yaw * self._guidance_yaw_gain

    def joystick_cb(self, msg: Joy):
        """Callback function that processes joy messages and converts them into wrench messages.

        This function sends wrench messages to the thrust allocation node. It handles
        the software killswitch and control mode buttons and transitions between different
        JoyStates of operation.

        Args:
            msg: A ROS message containing the joy input data.
        """
        self.check_number_of_buttons(msg)

        buttons: dict = self.populate_buttons_dictionary(msg)
        axes: dict = self.populate_axes_dictionary(msg)

        # Timetaking in precise nanoseconds to prevent double triggers on second transitions.
        current_time = self.get_clock().now().nanoseconds / 1e9

        xbox_control_mode_button = buttons.get("A", 0)
        software_killswitch_button = buttons.get("B", 0)
        software_control_mode_button = buttons.get("X", 0)
        reference_mode_button = buttons.get("Y", 0)

        self.calculate_movement(axes, buttons)

        if current_time - self._last_button_press_time < self._debounce_duration:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False
            reference_mode_button = False

        # Check if any button is pressed
        if any(
            [
                software_control_mode_button,
                xbox_control_mode_button,
                software_killswitch_button,
                reference_mode_button,
            ]
        ):
            self._last_button_press_time = current_time

        if software_killswitch_button:
            self.handle_killswitch_button()
        elif software_control_mode_button:
            self.transition_to_autonomous_mode()
        elif reference_mode_button:
            self.transition_to_reference_mode()
        elif xbox_control_mode_button:
            self.transition_to_xbox_mode()

        if not self._killswitch and self._mode in (
            OperationMode.MANUAL,
            OperationMode.REFERENCE,
        ):
            if self._mode == OperationMode.MANUAL:
                wrench_msg = self.create_wrench_message()
                self._wrench_publisher.publish(wrench_msg)
            else:
                self.update_reference()
                ref_msg = self._create_reference_msg()
                self._ref_publisher.publish(ref_msg)

            close = float(buttons.get("stick_button_left", 0))
            open_ = -float(buttons.get("stick_button_right", 0))
            rotate = axes.get("dpad_horizontal", 0.0)
            pitch = axes.get("dpad_vertical", 0.0)

            grip = close + open_

            gripper_msg = JointState()
            gripper_msg.header.stamp = self.get_clock().now().to_msg()
            gripper_msg.header.frame_id = "base_link"

            prefix = f"{self._drone}/"
            if self._drone == "orca":
                gripper_msg.name = [
                    f"{prefix}shoulder_joint",
                    f"{prefix}arm_joint",
                    f"{prefix}finger_joint1",
                    f"{prefix}finger_joint2",
                ]
                gripper_msg.velocity = [pitch, rotate, grip, grip]
            else:
                gripper_msg.name = [
                    f"{prefix}arm_joint",
                    f"{prefix}finger_joint1",
                    f"{prefix}finger_joint2",
                ]
                gripper_msg.velocity = [rotate, grip, grip]

            self._gripper_publisher.publish(gripper_msg)


def main():
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
