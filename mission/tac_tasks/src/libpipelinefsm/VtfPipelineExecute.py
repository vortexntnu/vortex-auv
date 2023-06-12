#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose

from libpipelinefsm.VtfPipelineFollowing import PipelineFollowing


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
        self.pipeline.state_pub.publish("pipeline/execute")

        while not rospy.is_shutdown():
            if not self.pipeline.task_manager_client.is_enabled:
                # Handles task change
                if self.pipeline.task_manager_client.was_enabled:
                    rospy.loginfo(f"STOPPING PIPELINE EXECUTE!")
                    self.is_logged = False
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

            elif object.isDetected or not self.pipeline.task_manager_client.was_enabled:
                self.pipeline.vtf_client.cancel_all_goals()
                goal = self.pipeline.goal
                goal.waypoints = [object.objectPose.pose.position]
                goal.forward_speed = 0.2
                goal.heading = "path_dependent_heading"
                self.pipeline.vtf_client.send_goal(goal)
                # Following has been initialized flag
                if not self.following_enabled:
                    self.following_enabled = True

                # Reset false counter if new measurement has been received
                self.false_detection_count = 0
                rospy.loginfo("Following next pipeline waypoint...")

            # After not detecting pipeline for some time, assume completed task
            elif self.following_enabled and not object.isDetected:
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
