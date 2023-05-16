import rospy
import rosnode
import subprocess

from geometry_msgs.msg import Wrench

from libjoystick.JoystickControlModes import *


class ControlModeHandling:

    def __init__(self):
        self.control_mode = JoystickControlModes(0)

    def control_mode_change(self, buttons, wrench_publisher_handle,
                            dp_client_handle):
        pressed = -1

        if buttons["stick_button_left"] and buttons[
                "stick_button_right"] and buttons["RB"] and buttons["LB"]:
            pressed = JoystickControlModes.KILLSWITCH.value
            self.control_mode = pressed
            ControlModeHandling.killswitch(buttons, wrench_publisher_handle)

        if buttons["start"]:
            pressed = JoystickControlModes.EMERGENCY_STOP.value
            self.control_mode = pressed
            ControlModeHandling.emergency_stop(wrench_publisher_handle)

        elif buttons["A"]:
            pressed = JoystickControlModes.OPEN_LOOP.value
            self.control_mode = pressed

        elif buttons["B"]:
            pressed = JoystickControlModes.POSE3_HOLD.value
            self.control_mode = pressed
            ControlModeHandling.pose3_hold(dp_client_handle)

        elif buttons["X"]:
            pressed = JoystickControlModes.POSE4_HOLD.value
            self.control_mode = pressed
            ControlModeHandling.pose4_hold(dp_client_handle)

        elif buttons["Y"]:
            pressed = JoystickControlModes.DP_CMD.value
            self.control_mode = pressed

        if pressed != -1:
            rospy.loginfo(
                f"Control mode changed by joystick: {get_joystick_control_mode_name(pressed)}"
            )
            rospy.sleep(
                rospy.Duration(0.25))  # Sleep to avoid aggressive switching

    @staticmethod
    def killswitch(buttons, wrench_publisher_handle):
        rospy.logwarn("KILLSWITCH ENABLED!")

        wrench_publisher_handle.publish(Wrench())
        rospy.sleep(rospy.Duration(1.0))

        nodes = rosnode.get_node_names()
        nodes_to_kill = [
            "/dp_controller_node", "thruster_interface", "pca9685_ros_driver"
        ]
        for node in nodes_to_kill:
            if node in nodes:
                ControlModeHandling.kill_node(node)
            else:
                rospy.loginfo(node + " is not running...")

    @staticmethod
    def emergency_stop(wrench_publisher_handle):
        rospy.logwarn("EMERGENCY STOP ENABLED!")
        nodes = rosnode.get_node_names()
        node = "/dp_controller_node"
        if node in nodes:
            ControlModeHandling.kill_node(node)
        else:
            rospy.loginfo(node + " is not running...")
        wrench_publisher_handle.publish(Wrench())
        rospy.sleep(rospy.Duration(1.0))

    @staticmethod
    def pose3_hold():
        rospy.logwarn("pose3_hold ENABLED")

    @staticmethod
    def pose4_hold():
        rospy.logwarn("pose4_hold ENABLED")

    # Function to kill a ROS node
    @staticmethod
    def kill_node(node_name):
        command = ['rosnode', 'kill', node_name]
        subprocess.call(command)
