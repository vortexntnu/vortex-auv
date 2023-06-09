#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose

from libpipelinefsm.PipelineFollowing import PipelineFollowing


class PipelineExecute(smach.State):

    def __init__(self, follow_depth, margin):
        smach.State.__init__(self, outcomes=["preempted", "succeeded"])

        self.pipeline = PipelineFollowing()
        self.pipeline.sending_rate = rospy.Rate(1)

        self.last_pose = Pose()
        self.is_logged = False
        self.pipeline.task_manager_client.is_enabled = True

        self.follow_depth = follow_depth
        self.margin = margin

        self.following_enabled = False
        self.false_detection_count = 0

    def execute(self, userdata):
        self.docking.state_pub.publish("docking/execute")

        while not rospy.is_shutdown():
            if not self.pipeline.task_manager_client.is_enabled:
                # Handles task change
                if self.pipeline.task_manager_client.was_enabled:
                    rospy.loginfo(f"STOPPING PIPELINE EXECUTE!")
                    self.is_logged = False
                    self.pipeline.dp_client.set_acceptance_margins(
                        [0.01, 0.01, 0.01, 0.0, 0.0, 10.0])
                    self.pipeline.dp_client.goal.x_ref = self.pipeline.odom_pose
                    self.pipeline.dp_client.send_goal()
                    self.pipeline.task_manager_client.was_enabled = False

                #self.pipeline.sending_rate.sleep() # TODO: REMOVE THIS COMMENT AFTER CONFIRMING THAT EVERYTHING ELSE WORKS!!!!!!!
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

            elif self.object.isDetected or not self.pipeline.task_manager_client.was_enabled:
                self.pipeline.dp_client.set_acceptance_margins(
                    [self.margin, self.margin, self.margin, 0.0, 0.0, 10.0])
                self.pipeline.dp_client.goal.x_ref.position.x = object_pos.x
                self.pipeline.dp_client.goal.x_ref.position.y = object_pos.y
                self.pipeline.dp_client.goal.x_ref.position.z = self.follow_depth
                self.pipeline.dp_client.goal.x_ref.orientation = object_rot
                if not self.pipeline.dp_client.get_enabled_status():
                    self.pipeline.dp_client.enable()
                self.pipeline.dp_client.send_goal()
                if not self.following_enabled:
                    self.following_enabled = True
                rospy.loginfo("Following next pipeline waypoint...")

            # After not detecting pipeline for some time, assume completed task
            elif self.following_enabled and not self.object.isDetected:
                self.false_detection_count += 1
                if self.false_detection_count >= 10:
                    return "succeeded"

            if not self.is_logged:
                self.pipeline.task_manager_client.was_enabled = True
                rospy.loginfo(f"STARTING PIPELINE EXECUTE!")
                rospy.sleep(rospy.Duration(1))
                self.is_logged = True

            self.pipeline.task_manager_client.was_enabled = True
            self.pipeline.sending_rate.sleep()
