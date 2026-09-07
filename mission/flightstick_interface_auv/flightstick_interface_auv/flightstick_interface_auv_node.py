#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, WrenchStamped
from rclpy.node import Node, Parameter
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from vortex_msgs.msg import OperationMode, ReferenceFilter, ReferenceFilterQuat
from vortex_msgs.srv import GetOperationMode, SetOperationMode, ToggleKillswitch
from vortex_utils.python_utils import PoseData
from vortex_utils_ros.qos_profiles import (
    reliable_profile,
    sensor_data_profile,
)
from vortex_utils_ros.ros_converter import pose_from_ros

from flightstick_interface_auv.flightstick_utils import (
    LogitechExtreme3DPro,
    modes,
)

start_message = r"""
+----------------------------------------------------+
|  Flightstick Interface                             |
|  Logitech Extreme 3D Pro  ->  6 DOF AUV control    |
+----------------------------------------------------+
"""


class FlightstickInterface(Node):
    """Manual and reference control of the AUV from a Logitech Extreme 3D Pro.

    Mirrors joystick_interface_auv, but for a flight stick instead of an Xbox
    pad. The stick has 4 analog axes against the pad's 6, so heave and roll
    move onto buttons and pitch onto the hat switch. Surge, sway and yaw stay
    proportional.
    """

    def __init__(self):
        super().__init__('flightstick_interface_auv')

        self.get_parameters()
        self.init_movement()
        self.init_auxiliary_state()
        self.set_publishers_and_subscribers()
        self.set_services()

        self._current_state = PoseData()
        self._desired_state = PoseData()
        self._current_quat = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w]
        self._desired_quat = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w]

        self._mode = OperationMode.MANUAL
        self._killswitch = True

        try:
            request = GetOperationMode.Request()
            future = self.get_operation_mode_client.call_async(request)
            future.add_done_callback(self.handle_initial_operation_mode_response)
        except Exception as e:
            self.get_logger().error(f"Failed to call GetOperationMode service: {e}")
            self._killswitch = True

        self._joystick_axes_map = LogitechExtreme3DPro.joystick_axes_map
        self._joystick_buttons_map = LogitechExtreme3DPro.joystick_buttons_map
        self._last_button_press_time = 0
        self._warned_wrong_device = False

        self.get_logger().info(start_message)

    def get_parameters(self):
        """Method to get the parameters from the config file."""
        self.declare_parameter('drone', Parameter.Type.STRING)
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
            'light_change_threshold',
            'deadzone_stick_x',
            'deadzone_stick_y',
            'deadzone_twist',
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
        self.declare_parameter('topics.guidance.dp_rpy', Parameter.Type.STRING)
        self.declare_parameter('topics.guidance.dp_quat', Parameter.Type.STRING)

        self._orientation_mode = self.get_parameter('orientation_mode').value
        if self._orientation_mode not in ('euler', 'quat'):
            self.get_logger().warn(
                f"Unknown orientation_mode '{self._orientation_mode}', defaulting to 'euler'"
            )
            self._orientation_mode = 'euler'

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

    def init_auxiliary_state(self):
        """State for the stubbed torpedo and light channels.

        Neither has a driver or a topic in vortex-auv yet, so these are
        logged rather than published. See the README for what needs to
        exist before they can be wired up for real.
        """
        self._torpedo_loaded = False
        self._prev_trigger = 0
        self._prev_thumb = 0
        self._light_level = None
        self._altitude_hold = False
        self._hold_depth = 0.0

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
            # The killswitch always starts engaged, whatever the manager
            # reports. The AUV must not be drivable until button 12 is
            # pressed deliberately.
            self._killswitch = True
            self.get_logger().info(
                f"Initial operation mode: {self._mode} | Killswitch forced on at startup"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """Callback function for the pose subscriber. Updates the current state of the AUV."""
        self._current_state = pose_from_ros(msg.pose.pose)
        q = msg.pose.pose.orientation
        self._current_quat = np.array([q.x, q.y, q.z, q.w])

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

    def create_reference_quat_message(self) -> ReferenceFilterQuat:
        """Creates a reference message with quaternion orientation from the desired state."""
        reference_msg = ReferenceFilterQuat()
        reference_msg.header.stamp = self.get_clock().now().to_msg()
        reference_msg.header.frame_id = "odom"
        reference_msg.x = self._desired_state.x
        reference_msg.y = self._desired_state.y
        reference_msg.z = self._desired_state.z
        reference_msg.qx = float(self._desired_quat[0])
        reference_msg.qy = float(self._desired_quat[1])
        reference_msg.qz = float(self._desired_quat[2])
        reference_msg.qw = float(self._desired_quat[3])
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

    def transition_to_manual_mode(self):
        """Signals that the operational mode has switched to manual wrench control."""
        request = SetOperationMode.Request()
        request.requested_operation_mode.operation_mode = OperationMode.MANUAL
        future = self.operation_mode_client.call_async(request)
        future.add_done_callback(self.operation_mode_response_callback)
        self.get_logger().info("Manual mode")

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
        self._desired_quat = self._current_quat.copy()
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

    def check_device_layout(self, msg: Joy) -> None:
        """Warns once if the connected device is not shaped like an Extreme 3D Pro."""
        if self._warned_wrong_device:
            return
        if (
            len(msg.axes) != LogitechExtreme3DPro.num_axes
            or len(msg.buttons) != LogitechExtreme3DPro.num_buttons
        ):
            self.get_logger().warning(
                f"Expected {LogitechExtreme3DPro.num_axes} axes and "
                f"{LogitechExtreme3DPro.num_buttons} buttons, got "
                f"{len(msg.axes)} and {len(msg.buttons)}. "
                "Is a Logitech Extreme 3D Pro connected?"
            )
            self._warned_wrong_device = True

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

    @staticmethod
    def apply_deadzone(value: float, deadzone: float) -> float:
        """Zeroes an axis inside the deadzone and rescales the rest to full travel.

        joy_node's own deadzone is a single global parameter applied to every
        analog axis, which is too blunt here: the twist axis rests off-centre
        and needs a generous deadzone, while the throttle lever driving the
        lights needs none at all. So run joy_node with deadzone 0.0 and
        deadzone each axis here instead.

        Args:
            value: Raw axis value in [-1.0, 1.0].
            deadzone: Half-width of the dead band, in the same units.

        Returns:
            The deadzoned value, still spanning the full [-1.0, 1.0] range.
        """
        if deadzone <= 0.0:
            return value
        if abs(value) < deadzone:
            return 0.0
        return (value - math.copysign(deadzone, value)) / (1.0 - deadzone)

    def calculate_movement(self, axes: dict, buttons: dict):
        """Calculates the 6 DOF movement vector based on flight stick input.

        Surge, sway and yaw come from the three analog stick axes, each with
        its own deadzone. Heave and roll come from button pairs and pitch from
        the hat, so those three are bang-bang: full gain or nothing.
        """
        heave_down = buttons.get("button_3", 0)
        heave_up = buttons.get("button_4", 0)
        roll_left = buttons.get("button_5", 0)
        roll_right = buttons.get("button_6", 0)

        stick_y = self.apply_deadzone(
            axes.get("vertical_axis_stick", 0.0), self._deadzone_stick_y
        )
        stick_x = self.apply_deadzone(
            axes.get("horizontal_axis_stick", 0.0), self._deadzone_stick_x
        )
        twist = self.apply_deadzone(
            axes.get("twist_axis_stick", 0.0), self._deadzone_twist
        )

        self.surge = stick_y * self._joystick_surge_gain
        self.sway = -stick_x * self._joystick_sway_gain
        self.heave = (heave_up - heave_down) * self._joystick_heave_gain
        self.roll = (roll_right - roll_left) * self._joystick_roll_gain
        # The hat is digital and bypasses joy_node's deadzone entirely.
        self.pitch = -axes.get("hat_vertical", 0.0) * self._joystick_pitch_gain
        self.yaw = -twist * self._joystick_yaw_gain

        if self._altitude_hold:
            # Depth is owned by the hold while it is active, so ignore the
            # heave buttons rather than fighting the held setpoint.
            self.heave = 0.0

    def handle_killswitch_button(self) -> None:
        """Handles the software killswitch button press."""
        request = ToggleKillswitch.Request()
        future = self.toggle_killswitch_client.call_async(request)
        future.add_done_callback(self.operation_mode_response_callback)

    def handle_altitude_hold(self) -> None:
        """Toggles altitude hold, latching the current depth on the way on.

        There is no HOLD_ALTITUDE value in vortex_msgs/OperationMode, so this
        is local to the node rather than a fourth operation mode: it pins the
        z component of the reference and ignores the heave buttons while
        active. It only has an effect in reference mode, where a depth
        setpoint exists to hold.
        """
        self._altitude_hold = not self._altitude_hold

        if self._altitude_hold:
            self._hold_depth = self._current_state.z
            self.get_logger().info(f"Altitude hold ON at z = {self._hold_depth:.2f} m")
            if self._mode != OperationMode.REFERENCE:
                self.get_logger().warning(
                    "Altitude hold only holds depth in reference mode; "
                    "in manual mode it just zeroes the heave buttons."
                )
        else:
            self.get_logger().info("Altitude hold OFF")

    def handle_torpedo(self, buttons: dict) -> None:
        """Stub for the torpedo channel -- logs intent, publishes nothing.

        vortex-auv has no torpedo topic, message or driver; the only trace of
        one is a raw `torpedo.gpio_pin` in beluga.yaml. Both actions fire on
        the rising edge so that holding a button does not repeat.
        """
        trigger = buttons.get("trigger", 0)
        thumb = buttons.get("thumb", 0)

        if thumb and not self._prev_thumb:
            self._torpedo_loaded = not self._torpedo_loaded
            state = "LOADED" if self._torpedo_loaded else "UNLOADED"
            self.get_logger().info(f"[stub] Torpedo {state}")

        if trigger and not self._prev_trigger:
            if self._torpedo_loaded:
                self.get_logger().info("[stub] Torpedo LAUNCH")
                self._torpedo_loaded = False
            else:
                self.get_logger().warning(
                    "[stub] Torpedo launch requested but nothing is loaded"
                )

        self._prev_trigger = trigger
        self._prev_thumb = thumb

    def handle_lights(self, axes: dict) -> None:
        """Stub for the light dimmer -- logs intent, publishes nothing.

        The throttle lever is absolute and does not self-centre, which suits a
        dimmer. It reads -1.0 at the bottom of its travel and +1.0 at the top,
        so brightness is the axis mapped onto 0..1.

        Note that joy_node's deadzone is a single global parameter applied to
        every analog axis. Any non-zero deadzone puts a flat spot in the middle
        of the lever's travel where brightness stops responding, so run
        joy_node with deadzone 0.0 and deadzone the stick axes downstream.
        """
        level = (axes.get("throttle_lever", -1.0) + 1.0) / 2.0
        level = min(max(level, 0.0), 1.0)

        if (
            self._light_level is None
            or abs(level - self._light_level) >= self._light_change_threshold
        ):
            self._light_level = level
            self.get_logger().info(f"[stub] Lights {level * 100:.0f}%")

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

        if self._orientation_mode == 'quat':
            rotation_matrix = Rotation.from_quat(self._desired_quat).as_matrix()
        else:
            rotation_matrix = self._desired_state.as_rotation_matrix()

        world_frame_vector = rotation_matrix @ body_frame_vector

        self._desired_state.x += world_frame_vector[0]
        self._desired_state.y += world_frame_vector[1]
        if self._altitude_hold:
            self._desired_state.z = self._hold_depth
        else:
            self._desired_state.z += world_frame_vector[2]

        if self._orientation_mode == 'quat':
            delta = Rotation.from_euler(
                'xyz',
                [
                    self.roll * self._guidance_roll_gain,
                    self.pitch * self._guidance_pitch_gain,
                    self.yaw * self._guidance_yaw_gain,
                ],
            )
            self._desired_quat = (
                Rotation.from_quat(self._desired_quat) * delta
            ).as_quat()
        else:
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
        self.check_device_layout(msg)

        buttons: dict = self.populate_buttons_dictionary(msg)
        axes: dict = self.populate_axes_dictionary(msg)

        # Timetaking in precise nanoseconds to prevent double triggers on second transitions.
        current_time = self.get_clock().now().nanoseconds / 1e9

        altitude_hold_button = buttons.get("button_8", 0)
        autonomous_mode_button = buttons.get("button_9", 0)
        reference_mode_button = buttons.get("button_10", 0)
        manual_mode_button = buttons.get("button_11", 0)
        software_killswitch_button = buttons.get("button_12", 0)

        self.calculate_movement(axes, buttons)
        self.handle_torpedo(buttons)
        self.handle_lights(axes)

        if current_time - self._last_button_press_time < self._debounce_duration:
            altitude_hold_button = False
            autonomous_mode_button = False
            reference_mode_button = False
            manual_mode_button = False
            software_killswitch_button = False

        # Check if any button is pressed
        if any(
            [
                altitude_hold_button,
                autonomous_mode_button,
                reference_mode_button,
                manual_mode_button,
                software_killswitch_button,
            ]
        ):
            self._last_button_press_time = current_time

        if software_killswitch_button:
            self.handle_killswitch_button()
        elif altitude_hold_button:
            self.handle_altitude_hold()
        elif autonomous_mode_button:
            self.transition_to_autonomous_mode()
        elif reference_mode_button:
            self.transition_to_reference_mode()
        elif manual_mode_button:
            self.transition_to_manual_mode()

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


def main():
    rclpy.init()
    flightstick_interface = FlightstickInterface()
    rclpy.spin(flightstick_interface)
    flightstick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
