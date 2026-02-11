#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, WrenchStamped
from rclpy.node import Node, Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from vortex_msgs.msg import OperationMode, ReferenceFilter
from vortex_msgs.srv import SetOperationMode, ToggleKillswitch
from vortex_utils.python_utils import PoseData
from vortex_utils_ros.qos_profiles import (
    reliable_profile,
    sensor_data_profile,
)
from vortex_utils_ros.ros_converter import pose_from_ros

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

        self._killswitch = True
        self._mode = OperationMode.AUTONOMOUS

        self._joystick_axes_map = []
        self._joystick_buttons_map = []
        self._last_button_press_time = 0

        self.get_logger().info(start_message)

    def get_parameters(self):
        """Method to get the parameters from the config file."""
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

        topic_params = ['pose', 'joy', 'wrench_input', 'killswitch', 'operation_mode']

        for param in topic_params:
            self.declare_parameter(f'topics.{param}', Parameter.Type.STRING)
            setattr(
                self,
                param + '_topic',
                self.get_parameter(f'topics.{param}').value,
            )

        self.declare_parameter('topics.guidance.dp', Parameter.Type.STRING)
        self.guidance_topic = self.get_parameter('topics.guidance.dp').value

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
        self._ref_publisher = self.create_publisher(
            ReferenceFilter, self.guidance_topic, qos_profile=best_effort_qos
        )

    def set_services(self):
        self.operation_mode_client = self.create_client(
            SetOperationMode, 'set_operation_mode'
        )

        while not self.operation_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Operation Mode service not available, waiting...')

        self.toggle_killswitch_client = self.create_client(
            ToggleKillswitch, 'toggle_killswitch'
        )

        while not self.toggle_killswitch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Toggle Killswitch service not available, waiting...'
            )

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
        reference_msg.header.frame_id = "odom"
        reference_msg.x = self._desired_state.x
        reference_msg.y = self._desired_state.y
        reference_msg.z = self._desired_state.z
        reference_msg.roll = self._desired_state.roll
        reference_msg.pitch = self._desired_state.pitch
        reference_msg.yaw = self._desired_state.yaw
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
        reference_msg = self.create_reference_message()
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
        """
        self._desired_state.x += self.surge * self._guidance_surge_gain
        self._desired_state.y += self.sway * self._guidance_sway_gain
        self._desired_state.z -= self.heave * self._guidance_heave_gain
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

        if self._mode == OperationMode.MANUAL and not self._killswitch:
            wrench_msg = self.create_wrench_message()
            self._wrench_publisher.publish(wrench_msg)

        elif self._mode == OperationMode.REFERENCE and not self._killswitch:
            self.update_reference()
            ref_msg = self.create_reference_message()
            self._ref_publisher.publish(ref_msg)


def main():
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
