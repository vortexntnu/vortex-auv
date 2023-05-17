#!/usr/bin/python3
# type: ignore

from enum import IntEnum

import rospy
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from vortex_msgs.msg import dpAction, dpGoal, dpResult
from geometry_msgs.msg import Wrench, Pose
from nav_msgs.msg import Odometry

from libjoystick.JoystickControlModes import *
from libjoystick.ControlModeHandling import ControlModeHandling


class JoystickInterface:

    def __init__(self):
        """
        Define constants used in the joystick mapping, and any ros
        specifics
        """
        rospy.init_node("joystick_interface")
        self.ros_rate = rospy.Rate(50.0)

        # Attributes
        self.buttons = {}
        self.axes = {}

        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.odom_pose = Pose()

        self.control_mode_handler = ControlModeHandling()

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

        # ROS params
        self.joystick_surge_scaling = rospy.get_param(
            "/joystick/scaling/surge", 60)
        self.joystick_sway_scaling = rospy.get_param("/joystick/scaling/sway",
                                                     60)
        self.joystick_heave_scaling = rospy.get_param(
            "/joystick/scaling/heave", 60)
        self.joystick_roll_scaling = rospy.get_param("/joystick/scaling/roll",
                                                     35)
        self.joystick_pitch_scaling = rospy.get_param(
            "/joystick/scaling/pitch", -30)
        self.joystick_yaw_scaling = rospy.get_param("/joystick/scaling/yaw",
                                                    20)

        # Subscribers and publishers
        self.joystick_sub = rospy.Subscriber("/joystick/joy",
                                             Joy,
                                             self.joystick_cb,
                                             queue_size=1)

        self.odom_sub = rospy.Subscriber("/odometry/filtered",
                                         Odometry,
                                         self.odom_cb,
                                         queue_size=1)

        self.joystick_pub = rospy.Publisher("/mission/joystick_data",
                                            Joy,
                                            queue_size=1)

        self.wrench_pub = rospy.Publisher(
            rospy.get_param("/thrust/thrust_topic"), Wrench, queue_size=1)

        # DP server and client
        dp_action_server = "DpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                      dpAction)

        # rospy.Subscriber("/DpAction/result", dpResult, self.dp_goal_cb)

        # Initialization
        rospy.loginfo("Waiting for joystick input...")
        rospy.wait_for_message("/joystick/joy", Joy)
        rospy.loginfo("Joystick interface is up and running")

    def joystick_cb(self, msg):
        try:
            self.buttons = {
                self.joystick_buttons_map[i]: msg.buttons[i]
                for i in range(len(msg.buttons))
            }
            self.axes = {
                self.joystick_axes_map[i]: msg.axes[i]
                for i in range(len(msg.axes))
            }
        except Exception as e:
            rospy.logerr(f"Error in joystick_cb: {e}")

    def odom_cb(self, msg):
        self.odom_pose = msg.pose.pose

    def publish_joystick_data(self):
        # Publish joystick and wrench data
        joystick_msg = Joy()
        joystick_msg.axes = [
            self.surge,
            self.sway,
            self.heave,
            self.roll,
            self.pitch,
            self.yaw,
            self.dpad_hor,
            self.dpad_ver,
        ]
        self.joystick_pub.publish(joystick_msg)

        if self.control_mode_handler.control_mode == JoystickControlModes.OPEN_LOOP.value:
            wrench_msg = Wrench()
            wrench_msg.force.x = self.surge
            wrench_msg.force.y = self.sway
            wrench_msg.force.z = self.heave
            wrench_msg.torque.x = self.roll
            wrench_msg.torque.y = self.pitch
            wrench_msg.torque.z = self.yaw
            self.wrench_pub.publish(wrench_msg)

    def spin(self):
        while not rospy.is_shutdown():
            if self.buttons != {}:
                self.control_mode_handler.control_mode_change(
                    self.buttons, self.wrench_pub, self.dp_client,
                    self.odom_pose)

            self.surge = self.axes[
                "vertical_axis_left_stick"] * self.joystick_surge_scaling
            self.sway = self.axes[
                "horizontal_axis_left_stick"] * self.joystick_sway_scaling
            self.heave = -(self.axes["RT"] -
                           self.axes["LT"]) / 2 * self.joystick_heave_scaling
            self.roll = (self.buttons["RB"] -
                         self.buttons["LB"]) * self.joystick_roll_scaling
            self.pitch = self.axes[
                "vertical_axis_right_stick"] * self.joystick_pitch_scaling * (
                    -1)
            self.yaw = self.axes[
                "horizontal_axis_right_stick"] * self.joystick_yaw_scaling

            self.dpad_hor = self.axes["dpad_horizontal"]
            self.dpad_ver = self.axes["dpad_vertical"]

            self.publish_joystick_data()

            self.ros_rate.sleep()


if __name__ == "__main__":
    try:
        joystick_interface = JoystickInterface()
        joystick_interface.spin()

    except rospy.ROSInterruptException:
        pass
