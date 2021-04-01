#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.

import rospy
import actionlib

from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus

from std_srvs.srv import SetBool

class DPGuidance:
    """
    Take an input goal from an action client and pass
    it on to the reference model, but this time on a topic.

    The goal is only passed on if dp is activated from the
    guidance interface.
    """

    def __init__(self):
        """
        Create the ROS node dp and set constants, as well as the action
        server that the fsm connects to. Connect to the move_base action
        server in the dp controller. The guidance and controller communicate
        through this server.
        """

        rospy.init_node('dp')

        self.period = 0.025 # Run at 40Hz
        self.controller_setpoint = Pose()

        self.is_active = False

        # Publisher for the reference model
        self.reference_model_pub = rospy.Publisher('/dp_guidance/output', Pose, queue_size=1)

        # Action server for receiving goal data
        self.goal_action_server = actionlib.SimpleActionServer(name='dp_action_server', ActionSpec=MoveBaseAction, auto_start=False)
        self.goal_action_server.register_goal_callback(self.goal_cb)  # Called whenever guidance_interface sends a new goal for the dp system
        self.goal_action_server.register_preempt_callback(self.preempt_cb)
        self.goal_action_server.start()

    def spin(self):
        """
        A replacement for the normal rospy.spin(), equivalent
        to "spinOnce" in roscpp (except simpler, since each cb/topic
        has it's own thread)
        """
        while not rospy.is_shutdown():
            if self.is_active:
                self.reference_model_pub.publish(controller_setpoint)

            rospy.sleep(rospy.Duration(self.period))


    def goal_cb(self):
        """
        Accept a goal from the guidance interface and store it as local state,
        then activate publishing
        """
        new_goal = self.goal_action_server.accept_new_goal()
        self.controller_setpoint = new_goal.target_pose

        self.is_active = True
        

    def preempt_cb(self):
        """
		The preempt callback for the action server.
		"""
        if self.goal_action_server.is_preempt_requested():
            rospy.loginfo("Goal action server in dp_guidance was preempted!")
            self.goal_action_server.set_preempted()

            self.is_active = False


if __name__ == '__main__':

    try:
        dp_guidance = DPGuidance()
        dp_guidance.spin()

    except rospy.ROSInternalException as e:
        rospy.logerr(e)