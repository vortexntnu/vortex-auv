#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped, Wrench
from vortex_msgs.msg import ReferenceFilter
from joystick_utils import Wired, WirelessXboxSeriesX, JoyStates
from vortex_utils.python_utils import PoseData
from vortex_utils.ros_converter import pose_from_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class JoystickInterface(Node):

    def __init__(self):
        super().__init__('joystick_interface_node')

        self.get_parameters()
        self.init_movement()
        self.set_publishers_and_subscribers()

        self.mode_ = JoyStates.KILLSWITCH

        self.current_state_ = PoseData()
        self.desired_state_ = PoseData()

        self.joystick_axes_map_ = []
        self.joystick_buttons_map_ = []
        self.last_button_press_time_ = 0

        self.get_logger().info(f"Joystick interface node started. Current mode: {self.mode_}")

    def get_parameters(self):
        params = ['joystick_surge_gain', 'joystick_sway_gain', 'joystick_heave_gain', 'joystick_roll_gain', 'joystick_pitch_gain', 'joystick_yaw_gain',
                  'guidance_surge_gain', 'guidance_sway_gain', 'guidance_heave_gain', 'guidance_roll_gain', 'guidance_pitch_gain', 'guidance_yaw_gain',
                  'debounce_duration']
        
        for param in params:
            self.declare_parameter(param, 1.0)
            setattr(self, param + '_', self.get_parameter(param).value)

    def init_movement(self):
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def set_publishers_and_subscribers(self):
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.joy_subscriber_ = self.create_subscription(
            Joy, "orca/joy", self.joystick_cb, 5)
        self.odom_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped, "/orca/pose", self.pose_cb, qos_profile=best_effort_qos)
        self.wrench_publisher_ = self.create_publisher(Wrench,
            "thrust/wrench_input", 10)
        self.ref_publisher = self.create_publisher(ReferenceFilter, "/dp/reference", 10)
        self.software_killswitch_signal_publisher_ = self.create_publisher(
            Bool, "softwareKillSwitch", 10)
        self.software_killswitch_signal_publisher_.publish(
            Bool(data=True))
        self.operational_mode_signal_publisher_ = self.create_publisher(
            String, "softwareOperationMode", 10)

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_state_ = pose_from_ros(msg)
    
    def create_reference_message(self) -> ReferenceFilter:
        reference_msg = ReferenceFilter()
        reference_msg.x = self.desired_state_.x
        reference_msg.y = self.desired_state_.y
        reference_msg.z = self.desired_state_.z
        reference_msg.roll = self.desired_state_.roll
        reference_msg.pitch = self.desired_state_.pitch
        reference_msg.yaw = self.desired_state_.yaw
        return reference_msg

    def create_wrench_message(self) -> Wrench:
        """
        Creates a 2D wrench message with the given x, y, heave, roll, pitch, and yaw values.

        Returns:
        Wrench: A 2D wrench message with the given values.
        """
        wrench_msg = Wrench()
        wrench_msg.force.x = self.surge
        wrench_msg.force.y = self.sway
        wrench_msg.force.z = self.heave
        wrench_msg.torque.x = self.roll
        wrench_msg.torque.y = self.pitch
        wrench_msg.torque.z = self.yaw
        return wrench_msg

    def transition_to_xbox_mode(self):
        """
        Turns off the controller and signals that the operational mode has switched to Xbox mode.
        """
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))
        self.mode_ = JoyStates.XBOX_MODE
        self.get_logger().info("XBOX mode")


    def transition_to_reference_mode(self):
        """
        Publishes a pose message and signals that the operational mode has switched to Reference mode.
        """
        self.desired_state_ = PoseData(
            x=self.current_state_.x,
            y=self.current_state_.y,
            z=self.current_state_.z,
            roll=self.current_state_.roll,
            pitch=self.current_state_.pitch,
            yaw=self.current_state_.yaw
        )
        reference_msg = self.create_reference_message()
        self.operational_mode_signal_publisher_.publish(String(data="Reference mode"))
        self.ref_publisher.publish(reference_msg)
        self.mode_ = JoyStates.REFERENCE_MODE
        self.get_logger().info("Reference mode")

    def transition_to_autonomous_mode(self):
        """
        Publishes a zero force wrench message and signals that the system is turning on autonomous mode.
        """
        empty_wrench_msg = Wrench()
        self.wrench_publisher_.publish(empty_wrench_msg)
        self.operational_mode_signal_publisher_.publish(
            String(data="autonomous mode"))
        self.mode_ = JoyStates.AUTONOMOUS_MODE
        self.get_logger().info("autonomous mode")

    def check_number_of_buttons(self, msg: Joy):
        """
        Checks if the controller is wireless (has 16 buttons) or wired and sets the joystick button and axis maps accordingly.
        
        Args:
            msg: A ROS message containing the joy input data.
        """        
        if len(msg.buttons) == 16:
            self.joystick_buttons_map_ = WirelessXboxSeriesX.joystick_buttons_map_
            self.joystick_axes_map_ = WirelessXboxSeriesX.joystick_axes_map_
        else:
            self.joystick_buttons_map_ = Wired.joystick_buttons_map_
            self.joystick_axes_map_ = Wired.joystick_axes_map_

    def populate_buttons_dictionary(self, msg: Joy):
        """
        Populates a dictionary with button JoyStates from the joystick message.
        
        Args:
            msg: A ROS message containing the joy input data.
        
        Returns:
            A dictionary with button names as keys and their JoyStates as values.
        """
        buttons = {}
        for i, button_name in enumerate(self.joystick_buttons_map_):
            if i < len(msg.buttons):
                buttons[button_name] = msg.buttons[i]
            else:
            # Assuming default value if button is not present
                buttons[button_name] = 0
        return buttons

    def populate_axes_dictionary(self, msg: Joy):
        """
        Populates a dictionary with axis values from the joystick message.
        
        Args:
            msg: A ROS message containing the joy input data.
        
        Returns:
            A dictionary with axis names as keys and their values as values.
        """
        axes = {}
        for i, axis_name in enumerate(self.joystick_axes_map_):
            if i < len(msg.axes):
                axes[axis_name] = msg.axes[i]
            else:
            # Assuming default value if axis is not present
                axes[axis_name] = 0.0
        return axes

    def calculate_movement (self, axes: dict, buttons: dict):
        """
        Calculates the movement values based on joystick input.
        """
        left_trigger = axes.get("LT", 0.0)
        right_trigger = axes.get("RT", 0.0)
        left_shoulder = buttons.get("LB", 0)
        right_shoulder = buttons.get("RB", 0)

        self.surge = axes.get(
            "vertical_axis_left_stick", 0.0
        ) * self.joystick_surge_gain_
        self.sway = -axes.get(
            "horizontal_axis_left_stick", 0.0
        ) * self.joystick_sway_gain_
        self.heave = (
            left_trigger - right_trigger
        ) * self.joystick_heave_gain_
        self.roll = (
            right_shoulder - left_shoulder
        ) * self.joystick_roll_gain_
        self.pitch = -axes.get(
            "vertical_axis_right_stick", 0.0
        ) * self.joystick_pitch_gain_
        self.yaw = -axes.get(
            "horizontal_axis_right_stick", 0.0
        ) * self.joystick_yaw_gain_

    def handle_killswitch_button(self) -> None:
        """
        Handles the software killswitch button press.

        This function performs the following actions based on the current state:
        1. If the current state is KILLSWITCH, it signals that the killswitch is not blocking,
            transitions to Xbox mode, and returns.
        2. Otherwise, it logs a message indicating that the software killswitch is active,
            signals that the killswitch is blocking, publishes a zero wrench message to stop
            the AUV, and sets the state to KILLSWITCH.

        The function ensures that the AUV stops moving when the killswitch is activated
        and allows it to resume operation when the killswitch is deactivated.
        """
        if self.mode_ == JoyStates.KILLSWITCH:
            self.software_killswitch_signal_publisher_.publish(
                Bool(data=False))
            self.transition_to_xbox_mode()
            return

        else:
            self.get_logger().info("SW killswitch")
            self.software_killswitch_signal_publisher_.publish(
                Bool(data=True))
            empty_wrench_msg = Wrench()
            self.wrench_publisher_.publish(empty_wrench_msg)
            self.mode_ = JoyStates.KILLSWITCH
            return
    
    def joystick_cb(self, msg: Joy):
        """
        Callback function that receives joy messages and converts them into
        wrench messages to be sent to the thruster allocation node. 
        Handles software killswitch and control mode buttons,
        and transitions between different JoyStates of operation.

        This function performs the following steps:
        1. Checks the number of buttons to set the correct joystick map.
        2. Populates dictionaries for button and axis JoyStates.
        3. Extracts specific button and axis values.
        4. Calculates movement values based on joystick input.
        5. Debounces button presses to prevent multiple triggers within a short duration.
        6. Updates the last button press time if any button is pressed.

        Args:
            msg: A ROS message containing the joy input data.
        """

        self.check_number_of_buttons(msg)

        buttons: dict = self.populate_buttons_dictionary(msg)
        axes: dict = self.populate_axes_dictionary(msg)
        current_time = self.get_clock().now().to_msg()._sec

        xbox_control_mode_button = buttons.get("A", 0)
        software_killswitch_button = buttons.get("B", 0)
        software_control_mode_button = buttons.get("X", 0)
        reference_mode_button = buttons.get("Y", 0)

        self.calculate_movement(axes, buttons)

        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False
            reference_mode_button = False

        if any([software_control_mode_button, xbox_control_mode_button, software_killswitch_button, reference_mode_button]):
            self.last_button_press_time_ = current_time

        if software_killswitch_button:
            self.handle_killswitch_button()

        if self.mode_ == JoyStates.XBOX_MODE:
            wrench_msg = self.create_wrench_message()
            self.wrench_publisher_.publish(wrench_msg)
            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode()

        elif self.mode_ == JoyStates.AUTONOMOUS_MODE:
            if xbox_control_mode_button:
                self.transition_to_xbox_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode() 

        elif self.mode_ == JoyStates.REFERENCE_MODE:
            self.update_reference()
            ref_msg = self.create_reference_message()
            ref_msg.header.stamp = self.get_clock().now().to_msg()
            ref_msg.header.frame_id = "odom"
            self.ref_publisher.publish(ref_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif xbox_control_mode_button:
                self.transition_to_xbox_mode()


    def update_reference(self): 
        """
        Updates the current pose of the AUV based on joystick inputs.
        The position and orientation (roll, pitch, yaw) are updated
        using the current joystick inputs scaled by their respective parameters.
        """
        self.desired_state_.x += self.surge / self.guidance_surge_gain_
        self.desired_state_.y += self.sway / self.guidance_sway_gain_
        self.desired_state_.z += self.heave / self.guidance_heave_gain_
        self.desired_state_.roll += self.roll / self.guidance_roll_gain_
        self.desired_state_.pitch += self.pitch / self.guidance_pitch_gain_
        self.desired_state_.yaw += self.yaw / self.guidance_yaw_gain_

def main():
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
