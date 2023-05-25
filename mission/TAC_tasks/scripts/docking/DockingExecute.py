#!/usr/bin/python3

import numpy as np

import rospy
import smach
import actionlib

from geometry_msgs.msg import Pose, Point, Wrench, Quaternion, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from landmarks.srv import request_position
from vortex_msgs.msg import (dpAction, dpGoal, dpResult, ObjectPosition)

import tf

from task_manager_defines import defines

import dynamic_reconfigure.client


class DockingExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        # TODO: Needs to be cleaned up and made into proper OOP code

        # Client to communicate with the task manager
        self.task_manager_client = dynamic_reconfigure.client.Client(
            "/task_manager/server",
            timeout=3,
            config_callback=self.task_manager_cb)

        self.isEnabled = False # Needs to default to standby
        self.task = "docking"

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("docking_point").object

        # Get TF from body to SPP
        tf_listener = tf.TransformListener()
        rospy.loginfo("Waiting for TF...")
        tf_listener.waitForTransform('SPP_link', 'base_link', rospy.Time(), rospy.Duration(1.0))

        # Enable Dp
        rospy.set_param("/DP/Enable", True)
        self.dp_client = actionlib.SimpleActionClient("/DpAction",
                                                      dpAction)
        self.dp_client.wait_for_server()

        rospy.Subscriber("/DpAction/result", dpResult, self.dp_goal_cb)
        self.reached_dp_goal = False
        self.current_goal_pose = Pose()

        # Odometry
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_pose_cb)
        self.odom_pose = Pose()
        self.SPP_pose = Pose()
        

        # Direct publisher to thrust for anchoring to the docking station
        self.thrust_pub = rospy.Publisher(
            rospy.get_param("/thrust/thrust_topic"), Wrench, queue_size=1)

        # Height to converge above the docking station
        self.convergence_height = rospy.get_param(
            "/tac/docking/convergence_height")

        # Height where we turn off DP for heave for quick docking to minimize issues with drift
        self.final_descent_height = rospy.get_param(
            "/tac/docking/final_descent_height")

    def execute(self, userdata):
        self.state_pub.publish("docking/execute")

        goal = dpGoal()
        goal.x_ref = self.object.objectPose.pose

        # Shifts DP goal from Docking_point to center of mass
        goal.x_ref.position.x = goal.x_ref.position.x + offsetPoint.x
        goal.x_ref.position.y = goal.x_ref.position.y + offsetPoint.y
        goal.x_ref.position.z = self.odom_pose.position.z

        # Set active degrees of freedom for DP
        goal.DOF = [True, True, True, False, False, True]

        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)
        self.current_goal_pose = goal.x_ref

        rate = rospy.Rate(10)
        sending_rate = rospy.Rate(1)
        sending_rate.sleep()

        while (not rospy.is_shutdown() and self.isEnabled
               and not self.reached_dp_goal):

            self.object = self.landmarks_client("docking_point").object
            goal.x_ref = self.object.objectPose.pose

            # Shift DP setpoint to account for position of powerpuck relative to origin in world frame
            offsetPoint = self.find_relative_to_mass_centre(powerPuckOffset)
            goal.x_ref.position.x = goal.x_ref.position.x + offsetPoint.x
            goal.x_ref.position.y = goal.x_ref.position.y + offsetPoint.y
            goal.x_ref.position.z = goal.x_ref.position.z - offsetPoint.z

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

        rospy.loginfo("BELUGA AT: " + str(self.odom_pose.position.x) +
                      "; " + str(self.odom_pose.position.y) + "; " +
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

    def task_manager_cb(self, config):
        activated_task_id = config["Tac_states"]

        if defines.Tasks.docking.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        rospy.logwarn(f"Docking Enabled: {self.isEnabled} ")

        return config

    def odom_cb(self, msg):
        self.odom_pose = msg.pose.pose

    def dp_goal_cb(self, msg):
        if (msg.result.finished):
            self.reached_dp_goal = True
