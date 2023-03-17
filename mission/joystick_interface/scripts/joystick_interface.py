#!/usr/bin/python3

from enum import IntEnum

import rospy
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from vortex_msgs.msg import ControlModeAction, ControlModeGoal
from geometry_msgs.msg import Wrench


class ControlModeEnum(IntEnum):
    OPEN_LOOP = 0
    POSITION_HOLD = 1
    HEADING_HOLD = 2
    DEPTH_HEADING_HOLD = 3
    DEPTH_HOLD = 4
    POSITION_HEADING_HOLD = 5
    CONTROL_MODE_END = 6
    POSE_HOLD = 7
    ORIENTATION_HOLD = 8
    ORIENTATION_DEPTH_HOLD = 9


class JoystickInterface:

    def __init__(self):
        """
        Define constants used in the joystick mapping, and any ros
        specifics
        """

        rospy.init_node("joystick_interface")

        self.joystick_surge_scaling = rospy.get_param(
            "/joystick/scaling/surge")
        self.joystick_sway_scaling = rospy.get_param("/joystick/scaling/sway")
        self.joystick_heave_scaling = rospy.get_param(
            "/joystick/scaling/heave")
        self.joystick_roll_scaling = rospy.get_param("/joystick/scaling/roll")
        self.joystick_pitch_scaling = rospy.get_param(
            "/joystick/scaling/pitch")
        self.joystick_yaw_scaling = rospy.get_param("/joystick/scaling/yaw")

        self.joystick_buttons_map = [
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

        self.joystick_axes_map = [
            "horizontal_axis_left_stick",
            "vertical_axis_left_stick",
            "LT",
            "horizontal_axis_right_stick",
            "vertical_axis_right_stick",
            "RT",
            "dpad_horizontal",
            "dpad_vertical",
        ]

        self.joystick_sub = rospy.Subscriber("/joystick/joy",
                                             Joy,
                                             self.joystick_cb,
                                             queue_size=1)
        self.joystick_pub = rospy.Publisher("/mission/joystick_data",
                                            Joy,
                                            queue_size=1)
        self.control_mode_pub = rospy.Publisher("/mission/control_mode",
                                                String,
                                                queue_size=1)

        self.wrench_pub = rospy.Publisher("/thrust/desired_forces",
                                          Wrench,
                                          queue_size=1)

        self.guidance_interface_client = actionlib.SimpleActionClient(
            "/guidance_interface/joystick_server", ControlModeAction)

        rospy.loginfo("Joystick interface is up and running")

    def joystick_cb(self, msg):
        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map[i]] = msg.axes[i]

        abxy = self._abxy_pressed(buttons)

        if abxy != -1:
            rospy.loginfo("Control mode changed by joystick: %d" % abxy)

            cm = ControlModeGoal()
            cm.controlModeIndex = abxy
            self.guidance_interface_client.send_goal(cm)

            control_mode_msg = String()
            if abxy == 0:
                control_mode_msg.data = "open loop"
            elif abxy == 7:
                control_mode_msg.data = "pose hold"
            elif abxy == 3:
                control_mode_msg.data = "depth heading hold"
            elif abxy == 1:
                control_mode_msg.data = "position hold"
            else:
                control_mode_msg.data = "unknown control mode"
            self.control_mode_pub.publish(control_mode_msg)

            rospy.sleep(
                rospy.Duration(0.25))  # Sleep to avoid aggressie switching

        surge = axes["vertical_axis_left_stick"] * self.joystick_surge_scaling
        sway = axes["horizontal_axis_left_stick"] * self.joystick_sway_scaling
        heave = (axes["RT"] - axes["LT"]) / 2 * self.joystick_heave_scaling
        roll = (buttons["RB"] - buttons["LB"]) * self.joystick_roll_scaling
        pitch = axes[
            "vertical_axis_right_stick"] * self.joystick_pitch_scaling * (-1)
        yaw = axes["horizontal_axis_right_stick"] * self.joystick_yaw_scaling

        dpad_lights = axes["dpad_horizontal"]
        dpad_gripper = axes["dpad_vertical"]

        joystick_msg = Joy()
        joystick_msg.axes = [
            surge,
            sway,
            heave,
            roll,
            pitch,
            yaw,
            dpad_lights,
            dpad_gripper,
        ]

        self.joystick_pub.publish(joystick_msg)

        wrench_msg = Wrench()
        wrench_msg.force.x = surge
        wrench_msg.force.y = sway
        wrench_msg.force.z = heave
        wrench_msg.torque.x = roll
        wrench_msg.torque.y = pitch
        wrench_msg.torque.z = yaw
        self.wrench_pub.publish(wrench_msg)

    def _abxy_pressed(self, buttons):
        pressed = -1

        if buttons["A"]:
            pressed = ControlModeEnum.OPEN_LOOP.value

        if buttons["B"]:
            pressed = ControlModeEnum.ORIENTATION_DEPTH_HOLD.value

        if buttons["X"]:
            pressed = ControlModeEnum.DEPTH_HOLD.value

        if buttons["Y"]:
            pressed = ControlModeEnum.POSITION_HEADING_HOLD.value

        return pressed


if __name__ == "__main__":
    try:
        joystick_interface = JoystickInterface()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
