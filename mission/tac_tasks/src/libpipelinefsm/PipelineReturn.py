#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose
from vortex_msgs.msg import dpAction, dpGoal, dpResult

from libpipelinefsm.PipelineFollowing import PipelineFollowing


class PipelineReturn(smach.State):

    def __init__(self, return_depth, margin):
        smach.State.__init__(self, outcomes=["succeeded"])

        self.pipeline = PipelineFollowing()
        self.odom = self.pipeline.odom_pose
        self.return_depth = return_depth
        self.margin = margin

        self.is_logged = False
        self.following_enabled = False

    def execute(self, userdata):
        self.pipeline.state_pub.publish("pipeline/return")

        while not rospy.is_shutdown():
            if not self.pipeline.task_manager_client.is_enabled:
                # Handles task change
                if self.pipeline.task_manager_client.was_enabled:
                    rospy.loginfo(f"STOPPING PIPELINE RETURN!")
                    self.is_logged = False
                    self.pipeline.dp_client.set_acceptance_margins([
                        self.margin, self.margin, self.margin, 0.0, 0.0, 10.0
                    ])
                    self.pipeline.dp_client.goal.x_ref = self.pipeline.odom_pose
                    self.pipeline.dp_client.send_goal()
                    self.pipeline.task_manager_client.was_enabled = False
                    return "succeeded"

            if self.is_logged:
                self.pipeline.dp_client.set_acceptance_margins(
                    [self.margin, self.margin, self.margin, 0.0, 0.0, 10.0])
                self.pipeline.dp_client.goal.x_ref.position.x = self.odom.position.x
                self.pipeline.dp_client.goal.x_ref.position.y = self.odom.position.y
                self.pipeline.dp_client.goal.x_ref.position.z = self.return_depth
                self.pipeline.dp_client.goal.x_ref.orientation = self.odom.orientation  #TODO: transform this correct
                if not self.pipeline.dp_client.get_enabled_status():
                    self.pipeline.dp_client.enable()
                self.pipeline.dp_client.send_goal()
                # Following has been initialized flag
                if not self.following_enabled:
                    self.following_enabled = True

                rospy.loginfo("I'm coming home!")

            elif not self.is_logged:
                self.pipeline.task_manager_client.was_enabled = True
                rospy.loginfo(f"STARTING PIPELINE RETURN!")
                rospy.sleep(rospy.Duration(1))
                self.is_logged = True

            self.pipeline.sending_rate.sleep()
