#!/usr/bin/python3

import numpy as np

import rospy
import smach

from geometry_msgs.msg import Pose, Wrench
from nav_msgs.msg import Odometry

from libdockingfsm.Docking import Docking

from libdockingfsm.Helpers import distance_between_points


class DockingConverge(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

        self.docking = Docking()
        self.docking.sending_rate = rospy.Rate(3)

        self.last_pose = Pose()
        self.is_logged = False

    def execute(self, userdata):
        self.docking.state_pub.publish("docking/converge")

        while not rospy.is_shutdown():
            if not self.docking.task_manager_client.is_enabled:
                # Handles task change
                if self.docking.task_manager_client.was_enabled:
                    rospy.loginfo("STOPPING DOCKING CONVERGE!")
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

            if 0.02 < distance_between_points(
                    object_pos, self.docking.dp_client.goal.x_ref.position,
                    self.docking.convergence_height
            ) or not self.docking.task_manager_client.was_enabled:
                self.docking.dp_client.set_acceptance_margins(
                    [0.5, 0.5, 0.5, 0.0, 0.0, 10.0])
                self.docking.dp_client.goal.x_ref.position.x = object_pos.x
                self.docking.dp_client.goal.x_ref.position.y = object_pos.y
                self.docking.dp_client.goal.x_ref.position.z = object_pos.z + self.docking.convergence_height
                self.docking.dp_client.goal.x_ref.orientation = object_rot
                if not self.docking.dp_client.get_enabled_status():
                    self.docking.dp_client.enable()
                self.docking.dp_client.send_goal()
                rospy.loginfo("Converging above docking point...")

            if not self.is_logged:
                self.docking.task_manager_client.was_enabled = True
                rospy.loginfo("STARTING DOCKING CONVERGE!")
                rospy.sleep(rospy.Duration(1))
                self.is_logged = True

            if self.docking.dp_client.has_reached_goal(
            ) and self.docking.task_manager_client.was_enabled:
                rospy.loginfo(
                    f"{rospy.get_name()}: Converged above docking point!")
                return 'succeeded'

            self.docking.sending_rate.sleep()
