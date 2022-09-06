#!/usr/bin/python3

# This search patterns looks like:
#
#     /     /     /
# ----  ----  ----   ...
#     \     \     \
#
# x -->

import rospy
import actionlib
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from vortex_msgs.msg import (
    VtfPathFollowingAction,
    VtfPathFollowingGoal,
)
from vortex_msgs.srv import SetVelocity
from landmarks.srv import request_position
from fsm_helper import (
    get_pose_in_front,
    rotate_certain_angle,
    within_acceptance_margins,
)

import numpy as np


class ForwardSweepSearch:
    def __init__(self, task):
        self.task = task
        self.odom = Odometry()
        self.rate = rospy.Rate(10)

        landmark_client = "send_positions"
        self.landmarks_client = rospy.ServiceProxy(landmark_client, request_position)
        self.object = self.landmarks_client(self.task).object
        rospy.wait_for_service(landmark_client)

        desired_velocity_topic = rospy.get_param(
            "/controllers/velocity_controller/desired_velocity_topic"
        )
        self.velocity_ctrl_client = rospy.ServiceProxy(
            desired_velocity_topic, SetVelocity
        )
        rospy.wait_for_service(desired_velocity_topic)

        self.vtf_client = actionlib.SimpleActionClient(
            "/controllers/vtf_action_server", VtfPathFollowingAction
        )

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)

    def odom_cb(self, msg):
        self.odom = msg

    def yaw_to_angle(self, angle):
        goal = Pose()
        goal.position = self.odom.pose.pose.position
        goal.orientation = self.odom.pose.pose.orientation
        goal = rotate_certain_angle(goal, angle)

        vel_goal = Twist()
        vel_goal.angular.z = np.sign(angle) * rospy.get_param("/fsm/turn_speed")
        vel_goal.linear.z = -0.01
        vel_goal.linear.x = 0.01

        self.velocity_ctrl_client(vel_goal, True)
        print(f"Searching for {self.task}, angle: ({angle}) ...")
        while not within_acceptance_margins(goal, self.odom, True):
            self.object = self.landmarks_client(self.task).object
            if self.object.isDetected:
                return True
            self.rate.sleep()

        self.velocity_ctrl_client(vel_goal, False)

        return False

    def run(self):
        # TODO: DP depth hold for the entire search pattern

        path_segment_counter = 1
        goal = VtfPathFollowingGoal()
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"

        initial_pose = self.odom.pose.pose

        while not self.object.isDetected:

            position_ahead = get_pose_in_front(
                initial_pose, path_segment_counter, 0
            ).position
            position_ahead.z = rospy.get_param("/fsm/operating_depth")
            goal.waypoints = [position_ahead]
            self.vtf_client.wait_for_server()
            self.vtf_client.send_goal(goal)

            while (
                self.vtf_client.simple_state
                != actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                self.object = self.landmarks_client(self.task).object
                if self.object.isDetected:
                    break
                self.rate.sleep()

            detection = self.yaw_to_angle(45)
            if detection:
                break

            detection = self.yaw_to_angle(-90)
            if detection:
                break

            detection = self.yaw_to_angle(45)
            if detection:
                break

            path_segment_counter += 1

        self.vtf_client.cancel_all_goals()
        return True
