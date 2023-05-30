#!/usr/bin/python3

import numpy as np

import rospy
import smach
import tf

from geometry_msgs.msg import Pose, Wrench

from libdockingfsm.Docking import Docking


class DockingExecute(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

        self.docking = Docking()

    def execute(self, userdata):
        self.docking.state_pub.publish("docking/execute")

        self.docking.dp_client.goal.x_ref = self.object.objectPose.pose

        # Set active degrees of freedom for DP
        self.docking.dp_client.goal.DOF = [True, True, True, False, False, True]

        self.docking.dp_client.send_goal()
        self.docking.current_goal_pose = self.docking.dp_client.goal.x_ref

        rate = rospy.Rate(10)
        sending_rate = rospy.Rate(1)
        sending_rate.sleep()

        while (not rospy.is_shutdown() and self.docking.task_manager_client.is_enabled
               and not self.reached_dp_goal):

            self.object = self.landmarks_client("docking_point").object
            goal.x_ref = self.object.objectPose.pose

            # If we are not above the doking station, we converge towards a point above it first
            if not self.above_docking_point():
                goal.x_ref.position.z = goal.x_ref.position.z + self.convergence_height
                rospy.loginfo("Converging above docking point")

            # If reached final descent height for the first time...
            elif ((self.odom_pose.position.z - goal.x_ref.position.z)
                  < self.final_descent_height):
                #Turn off DP
                rospy.set_param("/DP/Enable", False)

                # Start trusters pushing downwards
                downward_trust = rospy.get_param(
                    "/joystick/scaling/heave"
                ) * 0.5  # Half of max limit for joystick heave
                thrust_vector = Wrench()
                thrust_vector.force.z = -downward_trust * 0.5
                self.thrust_pub.publish(thrust_vector)

            if self.above_docking_point():
                rospy.logwarn("Converging to actual docking point")

            rospy.loginfo("DOCKING POINT DETECTED: " +
                          str(self.object.objectPose.pose.position.x) + ", " +
                          str(self.object.objectPose.pose.position.y) + ", " +
                          str(self.object.objectPose.pose.position.z))

            if self.should_send_new_goal():
                self.dp_client.wait_for_server()
                self.dp_client.send_goal(goal)
                rospy.loginfo("Sending DP goal")
                self.current_goal_pose = goal.x_ref

            sending_rate.sleep()

        self.reached_dp_goal = False
        rospy.set_param("/DP/Enable", False)

        if not self.isEnabled:
            return 'preempted'

        self.object = self.landmarks_client("docking_point").object

        rospy.loginfo("DOCKING POINT ESTIMATE CONVERGED AT: " +
                      str(self.object.objectPose.pose.position.x) + "; " +
                      str(self.object.objectPose.pose.position.y) + "; " +
                      str(self.object.objectPose.pose.position.z))

        rospy.loginfo("BELUGA AT: " + str(self.odom_pose.position.x) + "; " +
                      str(self.odom_pose.position.y) + "; " +
                      str(self.odom_pose.position.z))

        downward_trust = rospy.get_param(
            "/joystick/scaling/heave")  # Max limit for joystick heave
        thrust_vector = Wrench()
        thrust_vector.force.z = -downward_trust
        self.thrust_pub.publish(thrust_vector)

        rospy.loginfo("Docked to station")

        # Finds time 'docking_diration' seconds into the future and stays docked until that time
        docking_duration = rospy.get_param("/tac/docking/docking_duration")
        finished_docking_time = rospy.Time.now().to_sec() + docking_duration

        while (finished_docking_time > rospy.Time.now().to_sec()):
            rospy.loginfo("Waiting while docked")
            sending_rate.sleep()
            if not self.isEnabled:
                thrust_vector = Wrench()
                self.thrust_pub.publish(thrust_vector)
                return 'preempted'

        rospy.loginfo("Leaving docking station")

        # Turn off anchoring thrust
        thrust_vector = Wrench()
        self.thrust_pub.publish(thrust_vector)

        # Converge to standby height above the docking station
        undocking_pose = self.odom_pose
        standby_height = rospy.get_param("/tac/docking/standby_height")
        undocking_pose.position.z = undocking_pose.position.z + standby_height

        goal.x_ref = undocking_pose
        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)

        rospy.set_param("/DP/Enable", True)

        while (not rospy.is_shutdown() and self.isEnabled
               and not self.reached_dp_goal):
            rate.sleep()

        if not self.isEnabled:
            rospy.set_param("/DP/Enable", False)
            return 'preempted'

        return 'succeeded'

