#!/usr/bin/python3

import rospy
import smach

from geometry_msgs.msg import Pose, Wrench

from libdockingfsm.Docking import Docking
from libdockingfsm.Helpers import distance_between_points


# TODO: remake this into DockingReturn (home)
class DockingStandby(smach.State):

    def __init__(self, convergence_height=1.0, margin=0.5, is_enabled=False):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

        self.docking = Docking()
        self.docking.sending_rate = rospy.Rate(3)

        self.convergence_height = convergence_height
        self.margin = margin

        self.last_pose = Pose()
        self.is_logged = False
        self.docking.task_manager_client.is_enabled = is_enabled

    def execute(self, userdata):
        self.docking.state_pub.publish("docking/standby")

        while not rospy.is_shutdown():
            if not self.docking.task_manager_client.is_enabled:
                # Handles task change
                if self.docking.task_manager_client.was_enabled:
                    rospy.loginfo(f"STOPPING DOCKING STANDBY!")
                    self.is_logged = False
                    self.docking.dp_client.set_acceptance_margins(
                        [0.01, 0.01, 0.01, 0.0, 0.0, 10.0])
                    self.docking.dp_client.goal.x_ref = self.docking.odom_pose
                    self.docking.dp_client.send_goal()
                    self.docking.task_manager_client.was_enabled = False
                continue

            object = self.docking.landmarks_client("docking_point").object
            object_pos = object.objectPose.pose.position
            object_rot = object.objectPose.pose.orientation

            # Handle LM server bug where it sends 0 values if there is no object pose yet
            if abs(object_pos.x) == 0.0 and abs(object_pos.y) == 0.0 and abs(
                    object_pos.z) == 0.0:
                rospy.loginfo(
                    f"{rospy.get_name()}: No viable docking point yet...")
                self.docking.sending_rate.sleep()
                continue

            # Sends new DP goal on perception update or after task change
            if 0.02 < distance_between_points(
                    object_pos, self.docking.dp_client.goal.x_ref.position,
                    self.convergence_height
            ) or not self.docking.task_manager_client.was_enabled:
                self.docking.dp_client.set_acceptance_margins(
                    [self.margin, self.margin, self.margin, 0.0, 0.0, 10.0])
                self.docking.dp_client.goal.x_ref.position.x = object_pos.x
                self.docking.dp_client.goal.x_ref.position.y = object_pos.y
                self.docking.dp_client.goal.x_ref.position.z = object_pos.z + self.convergence_height
                self.docking.dp_client.goal.x_ref.orientation = object_rot
                if not self.docking.dp_client.get_enabled_status():
                    self.docking.dp_client.enable()
                self.docking.dp_client.send_goal()
                rospy.loginfo("Converging above docking point...")

            if not self.is_logged:
                self.docking.task_manager_client.was_enabled = True
                rospy.loginfo(
                    f"STARTING DOCKING CONVERGE TO HEIGHT {self.convergence_height}!"
                )
                rospy.sleep(rospy.Duration(1))
                self.is_logged = True

            if self.docking.dp_client.has_reached_goal(
            ) and self.docking.task_manager_client.was_enabled:
                rospy.loginfo(
                    f"{rospy.get_name()}: Converged {self.convergence_height}m above docking point!"
                )
                return "succeeded"

            self.docking.task_manager_client.was_enabled = True
            self.docking.sending_rate.sleep()
