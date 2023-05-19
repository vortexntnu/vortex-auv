#! /usr/bin/env python3

# TODO: move to dp_controller pkg as python library

import rospy
import actionlib

from vortex_msgs.msg import (dpAction, dpGoal, dpResult, dpFeedback)
from geometry_msgs.msg import Pose

class DPClient:

    def __init__(self):

        # Attributes
        self.is_enabled = False

        self.goal = dpGoal()

        # DP client
        self.client_handle = actionlib.SimpleActionClient(
            "/DpAction", dpAction)
        rospy.loginfo("Waiting for DP Server...")
        self.client_handle.wait_for_server()

        rospy.Subscriber("/DpAction/result", dpResult, self.result_cb)
        self.result = False

        rospy.Subscriber("/DpAction/feedback", dpFeedback, self.feedback_cb)
        self.feedback = [-999, -999, -999, -999, -999, -999]

    def get_enabled_status(self):
        try:
            self.is_enabled = rospy.get_param("/DP/Enable")
        except Exception:
            rospy.logwarn("Could not retrieve DP enabled status...")
        return self.is_enabled

    def enable(self):
        rospy.set_param("/DP/Enable", True)
        self.is_enabled = True

    def disable(self):
        rospy.set_param("/DP/Enable", False)
        self.is_enabled = False

    def send_goal(self):
        is_server = self.client_handle.wait_for_server(timeout=rospy.Duration(3))
        if not is_server:
            rospy.logwarn("Could not reach DP server...")
            return False
        
        # Error-handling
        if not (hasattr(self.goal.DOF, '__iter__') and hasattr(self.goal.DOF, '__len__')):
            raise ValueError("Attribute 'DOF' has to be an array-like structure.")
        elif not len(self.goal.DOF) == 6:
            raise ValueError("Attribute 'DOF' has to be of length 6.")
        elif not isinstance(self.goal.x_ref, Pose):
            raise ValueError("Attribute 'goal_pose' has to be an an instance of Pose.")
        rospy.loginfo(self.goal)
        self.client_handle.send_goal(self.goal)
        return True

    def result_cb(self, msg_bool):
        self.result = msg_bool

    def feedback_cb(self, float32_arr_msg):
        self.feedback = float32_arr_msg.data
