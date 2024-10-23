#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String, Float64MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion, Wrench
import numpy as np

class States:
    XBOX_MODE = 1
    AUTONOMOUS_MODE = 2
    NO_GO = 3
    REFERENCE_MODE = 4 

class Wired:
    joystick_buttons_map_ = [
        "A",
        "B",
        "X",
        "Y",
        "LB",
        "RB",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick", #Sway
        "vertical_axis_left_stick", #Surge
        "LT", #Negative thrust/torque multiplier
        "horizontal_axis_right_stick", #Yaw
        "vertical_axis_right_stick",
        "RT", #Positive thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
]

class WirelessXboxSeriesX:
    joystick_buttons_map_ = [
        "A",
        "B",
        "0",
        "X",
        "Y",
        "0",
        "LB",
        "RB",
        "0",
        "0",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
]

    joystick_axes_map_ = [
        "horizontal_axis_left_stick", #Sway
        "vertical_axis_left_stick", #Surge
        "horizontal_axis_right_stick", #Yaw
        "vertical_axis_right_stick",
        "RT", #Positive thrust/torque multiplier
        "LT", #Negative thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
]

class JoystickInterface(Node):

    def __init__(self):
        super().__init__('joystick_interface_node')
        self.get_logger().info("Joystick interface is up and running. \n When the XBOX controller is connected, press the killswitch button once to enter XBOX mode.")

        self.last_button_press_time_ = 0
        self.debounce_duration_ = 0.25
        self.state_ = States.NO_GO
        self.precise_manuevering_scaling_ = 1.0
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.joystick_buttons_map_ = []
        self.current_roll = 0.0
        self.current_pitch = 0.0    
        self.current_yaw = 0.0


        self.euler_angle_publisher = self.create_publisher(Float64MultiArray, "joystick/roll", 10)

        self.joystick_axes_map_ = []

        self.joy_subscriber_ = self.create_subscription(
        Joy, "joystick/joy", self.joystick_cb, 5)
        self.wrench_publisher_ = self.create_publisher(Wrench,
        "joystick/wrench",
        10) 
        self.pose_publisher = self.create_publisher(PoseStamped, "/dp/guidance", 10) 
        self.timer_ = self.create_timer(0.1, self.timer_cb) 


        self.declare_parameter('surge_scale_factor', 60.0)
        self.declare_parameter('sway_scale_factor', 60.0)
        self.declare_parameter('yaw_scale_factor', 60.0)
        self.declare_parameter('heave_scale_factor', 17.5)
        self.declare_parameter('roll_scale_factor', 30.0)
        self.declare_parameter('pitch_scale_factor', 20.0)

        #The scaling parameters for the reference mode
        self.declare_parameter('surge_scale_ref_param', 300)
        self.declare_parameter('sway_scale_ref_param', 300)
        self.declare_parameter('heave_scale_ref_param', 300)
        self.declare_parameter('yaw_scale_ref_param', 300)
        self.declare_parameter('roll_scale_ref_param', 300)
        self.declare_parameter('pitch_scale_ref_param', 300)

        #Gets the scaling factors from the yaml file
        self.joystick_surge_scaling_ = self.get_parameter(
        'surge_scale_factor').value
        self.joystick_sway_scaling_ = self.get_parameter(
        'sway_scale_factor').value
        self.joystick_yaw_scaling_ = self.get_parameter(
        'yaw_scale_factor').value
        self.joystick_heave_scaling_ = self.get_parameter(
        'heave_scale_factor').value
        self.joystick_roll_scaling_ = self.get_parameter(
        'roll_scale_factor').value
        self.joystick_pitch_scaling_ = self.get_parameter(
        'pitch_scale_factor').value

        #Gets the scaling factors for the reference mode from the yaml file
        self.joystick_surge_param_ = self.get_parameter(
        'surge_scale_ref_param').value
        self.joystick_sway_param_ = self.get_parameter(
        'sway_scale_ref_param').value
        self.joystick_heave_param_ = self.get_parameter(
        'heave_scale_ref_param').value
        self.joystick_yaw_param_ = self.get_parameter(
        'yaw_scale_ref_param').value
        self.joystick_roll_param_ = self.get_parameter(
        'roll_scale_ref_param').value
        self.joystick_pitch_param_ = self.get_parameter(
        'pitch_scale_ref_param').value

        #Killswitch publisher
        self.software_killswitch_signal_publisher_ = self.create_publisher(
            Bool, "softwareKillSwitch", 10)
        self.software_killswitch_signal_publisher_.publish(
            Bool(data=True)) #Killswitch is active

        #Operational mode publisher
        self.operational_mode_signal_publisher_ = self.create_publisher(
            String, "softwareOperationMode", 10) 
        self.current_pose = PoseStamped()


    def create_pose_message(self): 
        """
        Creates a PoseStamped message with the current pose of the AUV.

        This function creates a PoseStamped message, sets the current time as the timestamp,
        and assigns the current pose of the AUV to the pose field of the message.

        Returns:
            PoseStamped: A ROS PoseStamped message containing the current pose of the AUV.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose = self.current_pose.pose
        return pose_msg

    def create_wrench_message(self, surge: float, sway: float, heave: float, roll: float, pitch: float, yaw: float) -> Wrench:
        """
        Creates a 2D wrench message with the given x, y, heave, roll, pitch, and yaw values.

        Args:
        surge (float): The x component of the force vector.
        sway (float): The y component of the force vector.
        heave (float): The z component of the force vector.
        roll (float): The x component of the torque vector.
        pitch (float): The y component of the torque vector.
        yaw (float): The z component of the torque vector.

        Returns:
        Wrench: A 2D wrench message with the given values.
        """
        wrench_msg = Wrench()
        wrench_msg.force.x = surge
        wrench_msg.force.y = sway
        wrench_msg.force.z = heave
        wrench_msg.torque.x = roll
        wrench_msg.torque.y = pitch
        wrench_msg.torque.z = yaw
        return wrench_msg

    def transition_to_xbox_mode(self) -> None:
        """
        Turns off the controller and signals that the operational mode has switched to Xbox mode.
        """
        self.operational_mode_signal_publisher_.publish(String(data="XBOX"))
        self.state_ = States.XBOX_MODE
        self.get_logger().info("Transitioned to XBOX mode.")
        self.get_logger().info("XBOX mode")


    def transition_to_reference_mode(self):
        """
        Publishes a pose message and signals that the operational mode has switched to Reference mode.
        """
        pose_msg = self.create_pose_message()
        self.operational_mode_signal_publisher_.publish(String(data="Reference mode"))
        self.pose_publisher.publish(pose_msg)
        self.state_ = States.REFERENCE_MODE
        self.get_logger().info("Transitioned to reference mode")
        self.get_logger().info("Reference mode")



    def transition_to_autonomous_mode(self) -> None:
        """
        Publishes a zero force wrench message and signals that the system is turning on autonomous mode.
        """
        wrench_msg = self.create_wrench_message(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.wrench_publisher_.publish(wrench_msg)
        self.operational_mode_signal_publisher_.publish(
        String(data="autonomous mode"))
        self.state_ = States.AUTONOMOUS_MODE
        self.get_logger().info("autonomous mode")


    def check_number_of_buttons(self, msg: Joy) -> None:
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

    def populate_buttons_dictionary(self, msg: Joy) -> None:
        """
        Populates a dictionary with button states from the joystick message.
        
        Args:
            msg: A ROS message containing the joy input data.
        
        Returns:
            A dictionary with button names as keys and their states as values.
        """
        buttons = {}
        for i, button_name in enumerate(self.joystick_buttons_map_):
            if i < len(msg.buttons):
                buttons[button_name] = msg.buttons[i]
            else:
            # Assuming default value if button is not present
                buttons[button_name] = 0
        return buttons

    def populate_axes_dictionary(self, msg: Joy) -> None:
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

    def calculate_movement (self, axes: dict, left_trigger: float, right_trigger: float, left_shoulder: int, right_shoulder: int) -> None:
        """
        Calculates the movement values based on joystick input.

        Args:
            axes: A dictionary with axis names as keys and their states as values.
            left_trigger: The value of the left trigger.
            right_trigger: The value of the right trigger.
            left_shoulder: The state of the left shoulder button.
            right_shoulder: The state of the right shoulder button.
        """
        self.surge = axes.get(
        "vertical_axis_left_stick", 0.0
        ) * self.joystick_surge_scaling_ * self.precise_manuevering_scaling_
        self.sway = -axes.get(
        "horizontal_axis_left_stick", 0.0
        ) * self.joystick_sway_scaling_ * self.precise_manuevering_scaling_
        self.heave = (
        left_trigger - right_trigger
        ) * self.joystick_heave_scaling_ * self.precise_manuevering_scaling_
        self.roll = (
        right_shoulder - left_shoulder
        ) * self.joystick_roll_scaling_ * self.precise_manuevering_scaling_
        self.pitch = -axes.get(
        "vertical_axis_right_stick", 0.0
        ) * self.joystick_pitch_scaling_ * self.precise_manuevering_scaling_
        self.yaw = -axes.get(
        "horizontal_axis_right_stick", 0.0
        ) * self.joystick_yaw_scaling_ * self.precise_manuevering_scaling_

    def handle_killswitch_button(self) -> None:
        """
        Handles the software killswitch button press.

        This function performs the following actions based on the current state:
        1. If the current state is NO_GO, it signals that the killswitch is not blocking,
            transitions to Xbox mode, and returns.
        2. Otherwise, it logs a message indicating that the software killswitch is active,
            signals that the killswitch is blocking, publishes a zero wrench message to stop
            the AUV, and sets the state to NO_GO.

        The function ensures that the AUV stops moving when the killswitch is activated
        and allows it to resume operation when the killswitch is deactivated.
        """
        if self.state_ == States.NO_GO:
            # Signal that killswitch is not blocking
            self.software_killswitch_signal_publisher_.publish(
                Bool(data=False))
            self.transition_to_xbox_mode()
            return

        else:
            self.get_logger().info("SW killswitch",
            throttle_duration_sec=1)
            # Signal that killswitch is blocking
            self.software_killswitch_signal_publisher_.publish(
            Bool(data=True))
            # Publish a zero wrench message when killswitch is activated
            wrench_msg = self.create_wrench_message(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.wrench_publisher_.publish(wrench_msg)
            self.state_ = States.NO_GO
            return 

    def joystick_cb(self, msg: Joy) -> Wrench:
        """
        Callback function that receives joy messages and converts them into
        wrench messages to be sent to the thruster allocation node. 
        Handles software killswitch and control mode buttons,
        and transitions between different states of operation.

        This function performs the following steps:
        1. Checks the number of buttons to set the correct joystick map.
        2. Populates dictionaries for button and axis states.
        3. Extracts specific button and axis values.
        4. Calculates movement values based on joystick input.
        5. Debounces button presses to prevent multiple triggers within a short duration.
        6. Updates the last button press time if any button is pressed.

        Args:
            msg: A ROS message containing the joy input data.

        Returns:
            A ROS message containing the wrench data that was sent to the thruster allocation node.
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
        
        left_trigger = axes.get("RT", 0.0)
        right_trigger = axes.get("LT", 0.0)
        left_shoulder = buttons.get("LB", 0)
        right_shoulder = buttons.get("RB", 0)

        self.calculate_movement(axes, left_trigger, right_trigger, left_shoulder, right_shoulder)

        # Debounce for the buttons
        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False
            reference_mode_button = False

        # If any button is pressed, update the last button press time
        if software_control_mode_button or xbox_control_mode_button or software_killswitch_button or reference_mode_button:

            self.last_button_press_time_ = current_time

        # Toggle killswitch on and off
        if software_killswitch_button:
            self.handle_killswitch_button()

        if self.state_ == States.XBOX_MODE:
            wrench_msg = self.create_wrench_message(self.surge, self.sway, self.heave, 
                                                self.roll, self.pitch, self.yaw)
            self.wrench_publisher_.publish(wrench_msg)
            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode()

        elif self.state_ == States.AUTONOMOUS_MODE:

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()
            elif reference_mode_button:
                self.transition_to_reference_mode() 

        elif self.state_ == States.REFERENCE_MODE:

            if software_control_mode_button:
                self.transition_to_autonomous_mode()
            elif xbox_control_mode_button:
                self.transition_to_xbox_mode()

    def update_current_pose(self): 
        """
        Updates the current pose of the AUV based on joystick inputs.
        The position and orientation (roll, pitch, yaw) are updated
        using the current joystick inputs scaled by their respective parameters.
        """
        self.current_pose.pose.position.x += self.surge / self.joystick_surge_param_
        self.current_pose.pose.position.y += self.sway / self.joystick_sway_param_
        self.current_pose.pose.position.z += self.heave / self.joystick_heave_param_
        self.current_roll += self.roll / self.joystick_roll_param_
        self.current_pitch += self.pitch / self.joystick_pitch_param_
        self.current_yaw += self.yaw / self.joystick_yaw_param_

        self.current_roll = self.ssa(self.current_roll)
        self.current_pitch = self.ssa(self.current_pitch)
        self.current_yaw = self.ssa(self.current_yaw)

    def create_quaternion_msg(self) -> Quaternion:
        """
        Converts the current Euler angles (roll, pitch, yaw) to a quaternion.

        Returns:
        A Quaternion object representing the orientation.
        """
        q = self.euler_to_quat(self.current_roll, self.current_pitch, self.current_yaw)
        quaternion = Quaternion()
        quaternion.w = q[0]
        quaternion.x = q[1]
        quaternion.y = q[2]
        quaternion.z = q[3]
        return quaternion
    
    def reference_pose(self):
        """
        Updates and publishes the current pose of the AUV in reference mode.

        This function performs the following steps:
        1. Updates the current pose based on joystick inputs.
        2. Converts the current Euler angles (roll, pitch, yaw) to a quaternion.
        3. Sets the orientation of the current pose to the calculated quaternion.
        4. Publishes the updated pose to the pose publisher.

        This function is typically called periodically to ensure that the AUV's
        pose is continuously updated and published while in reference mode.
        """
        self.update_current_pose() 
        quaternion = self.create_quaternion_msg()
        self.current_pose.pose.orientation = quaternion
        self.pose_publisher.publish(self.current_pose)


    def timer_cb(self):
        """
        Timer callback function that is periodically called to update and publish the AUV's state.

        This function performs the following actions:
        1. Checks if the current state is REFERENCE_MODE.
        2. If in REFERENCE_MODE, it updates and publishes the current pose.
        3. Converts the current Euler angles (roll, pitch, yaw) to degrees.
        4. Publishes the Euler angles as a Float64MultiArray message.

        This function ensures that the AUV's pose and orientation are continuously updated
        and published while in reference mode.

        Note:
            This function is typically called periodically by a timer.
        """
        if self.state_ == States.REFERENCE_MODE:
            self.reference_pose()
            msg = self.create_pose_message() 
            self.pose_publisher.publish(msg)
            euler_msg = Float64MultiArray()
            roll_deg = self.current_roll * 180 / np.pi
            pitch_deg = self.current_pitch * 180 / np.pi
            yaw_deg = self.current_yaw * 180 / np.pi
            euler_msg.data = [roll_deg, pitch_deg, yaw_deg]
            self.euler_angle_publisher.publish(euler_msg)


    @staticmethod
    def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion.

        This function converts the provided Euler angles (in radians) to a quaternion
        representation. The quaternion is returned as a NumPy array with four elements:
        [w, x, y, z].

        Args:
            roll (float): The roll angle in radians.
            pitch (float): The pitch angle in radians.
            yaw (float): The yaw angle in radians.

        Returns:
            np.ndarray: A NumPy array representing the quaternion [w, x, y, z].
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return np.array([w, x, y, z])
    
    @staticmethod
    def ssa(angle: float) -> float:
        """
        Converts an angle from degrees to radians.

        Args:
        angle (float): The angle in degrees.

        Returns:
        float: The angle in radians.
        """
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

def main() -> None:
    """
    Initializes the ROS 2 client library, creates an instance of the JoystickInterface node,
    and starts spinning the node to process callbacks. Once the node is shut down, it destroys
    the node and shuts down the ROS 2 client library.

    This function is the entry point for the joystick interface application.
    """
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
