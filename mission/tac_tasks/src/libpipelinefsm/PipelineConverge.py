#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose

from libpipelinefsm.PipelineFollowing import PipelineFollowing


class PipelineConverge(smach.State):
    def __init__(self, follow_depth):
        smach.State.__init__(self, outcomes=["aborted", "succeeded"])

        self.pipeline = PipelineFollowing()
        self.pipeline.sending_rate = rospy.Rate(1)

        self.last_pose = Pose()
        self.is_logged = False
        self.pipeline.task_manager_client.is_enabled = True

        self.follow_depth = follow_depth

    def execute(self, userdata):
        self.pipeline.state_pub.publish("pipeline/standby")

        while not rospy.is_shutdown():
            if not self.pipeline.task_manager_client.is_enabled:
                # Handles task change
                if self.pipeline.task_manager_client.was_enabled:
                    rospy.loginfo(
                        f"STOPPING PIPELINE CONVERGE!"
                    )
                    self.is_logged = False
                    self.pipeline.dp_client.set_acceptance_margins(
                        [0.01, 0.01, 0.01, 0.0, 0.0, 10.0])
                    self.pipeline.dp_client.goal.x_ref = self.pipeline.odom_pose
                    self.pipeline.dp_client.send_goal()
                    self.pipeline.task_manager_client.was_enabled = False
                continue

            object = self.pipeline.landmarks_client("pipeline").object
            object_pos = object.objectPose.pose.position
            object_rot = object.objectPose.pose.orientation

            # Handle LM server bug where it sends 0 values if there is no object pose yet
            if abs(object_pos.x) == 0.0 and abs(object_pos.y) == 0.0 and abs(
                    object_pos.z) == 0.0:
                rospy.loginfo(
                    f"{rospy.get_name()}: No viable object pose yet...")
                self.pipeline.sending_rate.sleep()
                continue

            elif self.object.isDetected:
                self.follow_depth = self.pipeline.odom_pose.position.z
                return 'succeeded'

        self.pipeline.sending_rate.sleep()

        rospy.loginfo('PIPELINE FOLLOWING ENDED')
        return "aborted"
