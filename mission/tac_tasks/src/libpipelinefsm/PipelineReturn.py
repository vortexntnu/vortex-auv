#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose
from vortex_msgs.msg import dpAction, dpGoal, dpResult

from libpipelinefsm.PipelineFollowing import PipelineFollowing


class PipelineReturn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded"])

        self.pipeline = PipelineFollowing()
        self.pipeline.task_manager_client.is_enabled = is_enabled
        self.odom = self.pipeline

    def execute(self, userdata):
        self.pipeline.state_pub.publish("pipeline/return")

        # hold current position
        dp_goal = dpGoal()
        dp_goal.DOF = [True, True, True, False, False, True]
        dp_goal.x_ref = None #position is return area (0,0,-2)? yaw needs to be calculated.
        self.dp_client.send_goal(dp_goal)

        while not rospy.is_shutdown() and userdata.isEnabled:
            rospy.loginfo("RETURNING")
            if self.reached_goal:
                rospy.loginfo("RETURNED")
                return "succeded"
