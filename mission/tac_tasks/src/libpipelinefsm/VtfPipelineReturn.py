#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Point

from libpipelinefsm.VtfPipelineFollowing import PipelineFollowing


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
                    self.pipeline.task_manager_client.was_enabled = False
                    return "succeeded"

            home = Point()
            home.x = 0
            home.y = 0
            home.z = self.return_depth

            if self.is_logged and not self.following_enabled:
                self.pipeline.vtf_client.cancel_all_goals()
                goal = self.pipeline.goal
                goal.waypoints = [home]
                goal.forward_speed = 0.2
                goal.heading = "path_dependent_heading"
                self.pipeline.vtf_client.send_goal(goal)
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
