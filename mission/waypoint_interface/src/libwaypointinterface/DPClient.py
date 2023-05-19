#! /usr/bin/env python3

# TODO: move to dp_controller pkg as python library

import rospy
import actionlib

from vortex_msgs.msg import (dpAction, dpGoal, dpResult, dpFeedback)
from geometry_msgs.msg import Pose


class DPClient:
    """
    The DPClient class is designed to interact with a dynamic positioning (DP) server. 
    It initializes a SimpleActionClient and provides functions to enable and disable the 
    dynamic positioning system, send goals, and handle received results and feedback.
    """

    def __init__(self):
        """
        Initializes the DPClient node, attributes, action client, and subscribers.
        """

        # Attributes
        self.is_enabled = False

        self.goal = dpGoal()

        # DP client handle
        self.client_handle = actionlib.SimpleActionClient(
            "/DpAction", dpAction)
        rospy.loginfo("Waiting for DP Server...")
        self.client_handle.wait_for_server()

        rospy.Subscriber("/DpAction/result", dpResult, self.result_cb)
        self.result = False

        rospy.Subscriber("/DpAction/feedback", dpFeedback, self.feedback_cb)
        self.feedback = [-999, -999, -999, -999, -999, -999]

        self.get_enabled_status()


    def get_enabled_status(self):
        """
        Retrieves the current status of the DP system.

        Returns:
            bool: True if the DP system is enabled, False otherwise.
        """

        try:
            self.is_enabled = rospy.get_param("/DP/Enable")
        except Exception:
            rospy.logwarn("Could not retrieve DP enabled status...")
        return self.is_enabled

    def enable(self):
        """
        Enables the DP system.
        """
        rospy.set_param("/DP/Enable", True)
        self.is_enabled = True

    def disable(self):
        """
        Disables the DP system.
        """
        rospy.set_param("/DP/Enable", False)
        self.is_enabled = False

    def send_goal(self):
        """
        Sends the goal to the DP server.

        Returns:
            bool: True if the goal was sent successfully, False otherwise.
        """

        is_server = self.client_handle.wait_for_server(timeout=rospy.Duration(3))
        if not is_server:
            rospy.logwarn("Could not reach DP server...")
            return False

        # Error-handling
        if not (hasattr(self.goal.DOF, '__iter__')
                and hasattr(self.goal.DOF, '__len__')):
            raise ValueError(
                "Attribute 'DOF' has to be an array-like structure.")
        elif not len(self.goal.DOF) == 6:
            raise ValueError("Attribute 'DOF' has to be of length 6.")
        elif not isinstance(self.goal.x_ref, Pose):
            raise ValueError(
                "Attribute 'goal_pose' has to be an an instance of Pose.")
        rospy.loginfo(self.goal)
        self.client_handle.send_goal(self.goal)
        return True

    def result_cb(self, msg_bool):
        """
        Callback function for "/DpAction/result" topic.

        Args:
            msg_bool (Bool): The received Bool message.
        """
        self.result = msg_bool

    def feedback_cb(self, float32_arr_msg):
        """
        Callback function for "/DpAction/feedback" topic.
        Provides current error-to-goal for each degree-of-freedom (DOF).

        Args:
            float32_arr_msg (Float32[6]): The received Float32[6] message.
        """
        self.feedback = float32_arr_msg.data
