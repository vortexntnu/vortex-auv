#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Wrench
from vortex_msgs.msg import ReferenceFilter
from nav_msgs.msg import Odometry
from joystick_interface_auv.joystick_utils import *
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy, JointState
from joystick_interface_auv.joystick_interface import JoystickInterface

best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

class JoystickInterfaceNode(Node):

    def __init__(self):
        super().__init__('joystick_interface_auv_node')

        self.last_button_press_time_ = 0
        self.debounce_duration_ = 0.25

        self.mode_= JoyStates.KILLSWITCH
        self.reference_mode_ = "NED"

        self.current_state = State()
        self.desired_state = State()

        self.init_subs_and_pubs()

        self.gripper_desired_position_ = 0.0
        self.gripper_desired_rotation_ = 0.0
        self.gripper_grip_position_ = 0.0
        
        joystick_gains = self.get_joystick_gains()
        guidance_gains = self.get_guidance_gains()

        self.joystick_interface_ = JoystickInterface(joystick_gains, guidance_gains)

        self.get_logger().warn(f"Joystick interface initialized. Current mode: {self.mode_}")

    def get_joystick_gains(self) -> State:
        param_list = [
            'surge_scale_factor',
            'sway_scale_factor',
            'heave_scale_factor',
            'roll_scale_factor',
            'pitch_scale_factor',
            'yaw_scale_factor',
        ]

        for param in param_list:
            self.declare_parameter(param, 1.0)

        gains = State()
        for i, slot in enumerate(gains.__slots__):
            gains.__setattr__(slot, self.get_parameter(param_list[i]).value)
        
        return gains
    
    def get_guidance_gains(self) -> State:
        param_list = [
            'surge_scale_ref_param',
            'sway_scale_ref_param',
            'heave_scale_ref_param',
            'roll_scale_ref_param',
            'pitch_scale_ref_param',
            'yaw_scale_ref_param',
        ]

        for param in param_list:
            self.declare_parameter(param, 1.0)
        
        gains = State()
        for i, slot in enumerate(gains.__slots__):
            gains.__setattr__(slot, self.get_parameter(param_list[i]).value)
        
        return gains
    
    def init_subs_and_pubs(self):
        self.joy_subscriber_ = self.create_subscription(
            Joy, "orca/joy", self.joystick_cb, 5)
        self.odom_subscriber_ = self.create_subscription(
            Odometry, "/orca/odom", self.odom_cb, qos_profile=best_effort_qos)
        self.wrench_publisher_ = self.create_publisher(
            Wrench, "thrust/wrench_input",5) 
        
        self.ref_publisher = self.create_publisher(
            ReferenceFilter, "/dp/reference", qos_profile=best_effort_qos)

        self.gripper_pos_publisher_ = self.create_publisher(
            Float64, "orca/gripper_cmd", 10)

        self.gripper_rot_publisher_ = self.create_publisher(
            Float64, "orca/gripper_arm_cmd", 10)

        self.gripper_finger_publisher_ = self.create_publisher(
            Float64, "orca/gripper_finger_cmd", 10)

        self.gripper_state_publisher_ = self.create_publisher(
            JointState, "stonefish/servos", 10)
        
        self.software_killswitch_signal_publisher_ = self.create_publisher(
            Bool, "softwareKillSwitch", 10)
        
        self.operational_mode_signal_publisher_ = self.create_publisher(
            String, "softwareOperationMode", 10) 

    def odom_cb(self, msg: Odometry):
        self.current_state.from_odom_msg(msg)

    def transition_to_xbox_mode(self):
        """
        Turns off the controller and signals that the operational mode has switched to Xbox mode.
        """
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))
        self.mode_= JoyStates.XBOX_MODE
        self.get_logger().warn("XBOX mode")


    def transition_to_reference_mode(self):
        """
        Publishes a pose message and signals that the operational mode has switched to Reference mode.
        """
        self.desired_state = State(
            x=self.current_state.x,
            y=self.current_state.y,
            z=self.current_state.z,
            roll=self.current_state.roll,
            pitch=self.current_state.pitch,
            yaw=self.current_state.yaw
        )
        reference_msg = self.desired_state.to_reference_msg()
        self.operational_mode_signal_publisher_.publish(String(data="Reference mode"))
        self.ref_publisher.publish(reference_msg)
        self.mode_= JoyStates.REFERENCE_MODE
        self.get_logger().warn(f"Reference mode: {self.reference_mode_}")

    def transition_to_autonomous_mode(self):
        """
        Publishes a zero force wrench message and signals that the system is turning on autonomous mode.
        """
        empty_wrench_msg = Wrench()
        self.wrench_publisher_.publish(empty_wrench_msg)
        self.operational_mode_signal_publisher_.publish(
            String(data="autonomous mode"))
        self.mode_= JoyStates.AUTONOMOUS_MODE
        self.get_logger().warn("autonomous mode")

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

    def populate_buttons_dictionary(self, msg: Joy) -> dict:
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
    

    def populate_axes_dictionary(self, msg: Joy) -> dict:
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

    def handle_killswitch_button(self):
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
        if self.mode_== JoyStates.KILLSWITCH:
            self.software_killswitch_signal_publisher_.publish(
                Bool(data=False))
            self.transition_to_xbox_mode()
            return

        else:
            self.get_logger().warn("SW killswitch")
            self.software_killswitch_signal_publisher_.publish(
            Bool(data=True))

            empty_wrench_msg = Wrench()
            self.wrench_publisher_.publish(empty_wrench_msg)
            self.mode_= JoyStates.KILLSWITCH
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

        #Check the number of buttons and axes
        self.check_number_of_buttons(msg)

        #Populate the buttons and axes dictionaries
        buttons: dict = self.populate_buttons_dictionary(msg)
        axes: dict = self.populate_axes_dictionary(msg)
        current_time = self.get_clock().now().to_msg()._sec

        # Extract button values
        xbox_control_mode_button = buttons.get("A", 0)
        software_killswitch_button = buttons.get("B", 0)
        software_control_mode_button = buttons.get("X", 0)
        reference_mode_button = buttons.get("Y", 0)
        
        gripper_move = axes.get("dpad_vertical", 0.0)
        gripper_rotation = axes.get("dpad_horizontal", 0.0)
        gripper_grip = buttons.get("stick_button_left", 0)
        gripper_open = buttons.get("stick_button_right", 0)

        movement: State = self.joystick_interface_.calculate_movement(buttons, axes)

        # Debounce for the buttons
        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False
            reference_mode_button = False

        # If any button is pressed, update the last button press time
        if any([software_control_mode_button, xbox_control_mode_button, software_killswitch_button, reference_mode_button]):
            self.last_button_press_time_ = current_time

        # Toggle killswitch on and off
        if software_killswitch_button:
            self.handle_killswitch_button()
            
        if not self.mode_== JoyStates.KILLSWITCH:
            self.handle_gripper(gripper_move, gripper_rotation, gripper_grip, gripper_open)

        if self.mode_== JoyStates.XBOX_MODE:
            wrench_msg = movement.to_wrench_msg()
            self.wrench_publisher_.publish(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode()

        elif self.mode_== JoyStates.AUTONOMOUS_MODE:

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode() 

        elif self.mode_== JoyStates.REFERENCE_MODE:
            if self.reference_mode_ == "BODY":
                movement_arr = movement.as_pos_array()
                rotation_matrix = self.current_state.as_rotation_matrix()
                movement_body = rotation_matrix.T @ movement_arr
                movement = State()
                movement.from_pos_array(movement_body)

            self.desired_state = self.joystick_interface_.update_reference_state(self.desired_state, movement)
            msg = self.desired_state.to_reference_msg()
            self.ref_publisher.publish(msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif xbox_control_mode_button:
                self.transition_to_xbox_mode()
            elif reference_mode_button:
                if self.reference_mode_ == "BODY":
                    self.reference_mode_ = "NED"
                else:
                    self.reference_mode_ = "BODY"
                self.get_logger().warn(f"Reference mode: {self.reference_mode_}")


    def handle_gripper(self, gripper_move: float, gripper_rotation: float, gripper_grip: bool, gripper_open: bool):
        gripper_state_msg = JointState()
        gripper_names = [
            "Orca/Shoulder_joint", "Orca/Arm_joint", "Orca/Finger_joint1",
            "Orca/Finger_joint2"
        ]
        gripper_pos = []

        gripper_msg = Float64()
        self.gripper_desired_position_ += gripper_move * 0.01
        if self.gripper_desired_position_ > .11:
            self.gripper_desired_position_ = .11
        if self.gripper_desired_position_ < -2.1:
            self.gripper_desired_position_ = -2.1

        gripper_pos.append(self.gripper_desired_position_)

        gripper_state_msg.name = gripper_names

        gripper_msg.data = self.gripper_desired_position_
        self.gripper_pos_publisher_.publish(gripper_msg)

        gripper_rot_msg = Float64()
        self.gripper_desired_rotation_ += gripper_rotation * 0.05
        gripper_rot_msg.data = self.gripper_desired_rotation_
        self.gripper_rot_publisher_.publish(gripper_rot_msg)

        gripper_finger_msg = Float64()
        if gripper_grip:
            self.gripper_grip_position_ += 0.01
        if gripper_open:
            self.gripper_grip_position_ -= 0.01
        if self.gripper_grip_position_ < 0:
            self.gripper_grip_position_ = 0.
        elif self.gripper_grip_position_ > 0.78:
            self.gripper_grip_position_ = 0.78

        gripper_pos.append(self.gripper_desired_rotation_)
        gripper_pos.append(self.gripper_grip_position_)
        gripper_pos.append(self.gripper_grip_position_)

        gripper_finger_msg.data = self.gripper_grip_position_
        self.gripper_finger_publisher_.publish(gripper_finger_msg)

        gripper_state_msg.position = gripper_pos

        self.gripper_state_publisher_.publish(gripper_state_msg)

    
def main():
    rclpy.init()
    joystick_interface = JoystickInterfaceNode()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()