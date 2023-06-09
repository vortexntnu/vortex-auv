#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose

from libpipelinefsm.PipelineFollowing import PipelineFollowing


class PipelineConverge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["aborted", "succeeded"])

        self.pipeline = PipelineFollowing()
        self.pipeline.sending_rate = rospy.Rate(1)

        self.last_pose = Pose()
        self.is_logged = False
        self.pipeline.task_manager_client.is_enabled = is_enabled

    def execute(self, userdata):
        self.pipeline.state_pub.publish("docking/standby")

        # hold current position
        dp_goal = dpGoal()
        dp_goal.DOF = [True, True, True, False, False, True]
        dp_goal.x_ref = self.odom.pose.pose
        self.dp_client.send_goal(dp_goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and userdata.isEnabled:
            rospy.loginfo("Standby")
            self.object = self.landmarks_client(
                self.task).object  # requesting update on the object
            self.isDetected = self.object.isDetected
            if self.isDetected:
                return "succeeded"
            rate.sleep()

        rospy.loginfo('PIPELINE FOLLOWING ENDED')
        return "aborted"
