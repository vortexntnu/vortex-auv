#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class States:
    XBOX_MODE = 1
    AUTONOMOUS_MODE = 2
    NO_GO = 3


class JoystickInterface(Node):

    def __init__(self):
        super().__init__('joystick_interface_node')
        self.get_logger().info(
            "Joystick interface is up and running. \n When the XBOX controller is connected, press the killswitch button once to enter XBOX mode."
        )

        self.last_button_press_time_ = 0
        self.debounce_duration_   = 0.25
        self.state_       = States.NO_GO

        self.joystick_buttons_map_ = [
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
        ]

        self.joystick_axes_map_ = [
            "horizontal_axis_left_stick",  #Translation (Left and Right)
            "vertical_axis_left_stick",  #Translation (Forwards and Backwards)
            "LT",  #Negative thrust/torque multiplier
            "horizontal_axis_right_stick",  #Rotation
            "vertical_axis_right_stick",
            "RT",  #Positive thrust/torque multiplier
            "dpad_horizontal",
            "dpad_vertical",
        ]

        self.joy_subscriber_ = self.create_subscription(
            Joy, "joystick/joy", self.joystick_cb, 1)
        self.wrench_publisher_ = self.create_publisher(Wrench,
                                                       "thrust/wrench_input",
                                                       1)

        self.declare_parameter('surge_scale_factor', 60.0)
        self.declare_parameter('sway_scale_factor', 60.0)
        self.declare_parameter('yaw_scale_factor', 60.0)
        self.declare_parameter('heave_scale_factor', 35.0)
        self.declare_parameter('roll_scale_factor', -30.0)
        self.declare_parameter('pitch_scale_factor', 20.0)

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

        #Killswitch publisher
        self.software_killswitch_signal_publisher_ = self.create_publisher(
            Bool, "softWareKillSwitch", 10)
        self.software_killswitch_signal_publisher_.publish(
            Bool(data=False))  #Killswitch is not active

        #Operational mode publisher
        self.operational_mode_signal_publisher_ = self.create_publisher(
            Bool, "softWareOperationMode", 10)

        # Signal that we are not in autonomous mode
        self.operational_mode_signal_publisher_.publish(Bool(data=True))

        #Controller publisher
        self.enable_controller_publisher_ = self.create_publisher(
            Bool, "controller/lqr/enable", 10)

    def create_wrench_message(self, surge: float, sway: float, heave: float,
                                 roll: float, pitch: float,
                                 yaw: float) -> Wrench:
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

    def transition_to_xbox_mode(self):
        """
        Turns off the controller and signals that the operational mode has switched to Xbox mode.
        """
        self.enable_controller_publisher_.publish(Bool(data=False))
        self.operational_mode_signal_publisher_.publish(Bool(data=True))
        self.state_ = States.XBOX_MODE

    def transition_to_autonomous_mode(self):
        """
        Publishes a zero force wrench message and signals that the system is turning on autonomous mode.
        """
        wrench_msg = self.create_wrench_message(0.0, 0.0, 0.0, 0.0, 0.0,
                                                   0.0)
        self.wrench_publisher_.publish(wrench_msg)
        self.operational_mode_signal_publisher_.publish(Bool(data=False))
        self.state_ = States.AUTONOMOUS_MODE

    def joystick_cb(self, msg: Joy) -> Wrench:
        """
        Callback function that receives joy messages and converts them into
        wrench messages to be sent to the thruster allocation node. 
        Handles software killswitch and control mode buttons,
        and transitions between different states of operation.

        Args:
            msg: A ROS message containing the joy input data.

        Returns:
            A ROS message containing the wrench data that was sent to the thruster allocation node.
        """

        current_time = self.get_clock().now().to_msg()._sec

        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map_[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map_[i]] = msg.axes[i]

        xbox_control_mode_button = buttons["A"]
        software_killswitch_button = buttons["B"]
        software_control_mode_button = buttons["X"]
        left_trigger = axes["RT"]
        right_trigger = axes["LT"]
        left_shoulder = buttons["LB"]
        right_shoulder = buttons["RB"]

        surge = axes[
            "vertical_axis_left_stick"] * self.joystick_surge_scaling_
        sway = axes[
            "horizontal_axis_left_stick"] * self.joystick_sway_scaling_
        heave = -(left_trigger - right_trigger)/2 * self.joystick_heave_scaling_
        roll = (right_shoulder - left_shoulder) * self.joystick_roll_scaling_
        pitch = -axes[
            "vertical_axis_right_stick"] * self.joystick_pitch_scaling_
        yaw = axes["horizontal_axis_right_stick"] * self.joystick_yaw_scaling_

        # Debounce for the buttons
        if current_time - self.last_button_press_time_ < self.debounce_duration_:
            software_control_mode_button = False
            xbox_control_mode_button = False
            software_killswitch_button = False

        # If any button is pressed, update the last button press time
        if software_control_mode_button or xbox_control_mode_button or software_killswitch_button:
            self.last_button_press_time_ = current_time

        # Toggle ks on and off
        if self.state_ == States.NO_GO and software_killswitch_button:
            # signal that killswitch is not blocking
            self.software_killswitch_signal_publisher_.publish(Bool(data=True))
            self.transition_to_xbox_mode()
            return

        if software_killswitch_button:
            self.get_logger().info("SW killswitch", throttle_duration_sec=1)
            # signal that killswitch is blocking
            self.software_killswitch_signal_publisher_.publish(
                Bool(data=False))
            # Turn off controller in sw killswitch
            self.enable_controller_publisher_.publish(Bool(data=False))
            # Publish a zero wrench message when sw killing
            wrench_msg = self.create_wrench_message(0.0, 0.0, 0.0, 0.0, 0.0,
                                                       0.0)
            self.wrench_publisher_.publish(wrench_msg)
            self.state_ = States.NO_GO
            return wrench_msg

        #Msg published from joystick_interface to thrust allocation
        wrench_msg = self.create_wrench_message(surge, sway, heave, roll,
                                                   pitch, yaw)

        if self.state_ == States.XBOX_MODE:
            self.get_logger().info("XBOX mode", throttle_duration_sec=1)
            self.wrench_publisher_.publish(wrench_msg)

            if software_control_mode_button:
                self.transition_to_autonomous_mode()

        if self.state_ == States.AUTONOMOUS_MODE:
            self.get_logger().info("autonomous mode", throttle_duration_sec=1)

            if xbox_control_mode_button:
                self.transition_to_xbox_mode()

        return wrench_msg


def main():
    rclpy.init()
    joystick_interface = JoystickInterface()
    rclpy.spin(joystick_interface)
    joystick_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
