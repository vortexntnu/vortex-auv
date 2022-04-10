#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.


from enum import IntEnum

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseActionGoal
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)
from std_srvs.srv import SetBool

from vortex_msgs.msg import DpSetpoint
from vortex_msgs.srv import ControlMode


class ControlModeEnum(IntEnum):
    OPEN_LOOP = 0
    POSITION_HOLD = 1
    HEADING_HOLD = 2
    DEPTH_HEADING_HOLD = 3
    DEPTH_HOLD = 4
    POSITION_HEADING_HOLD = 5
    CONTROL_MODE_END = 6
    POSE_HOLD = 7
    ORIENTATION_HOLD = 8
    ORIENTATION_DEPTH_HOLD = 9


def within_acceptance_margins(setpoint, odom_msg):
        acceptance_margins = rospy.get_param(
            "/guidance/dp/acceptance_margins", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        )
        acceptance_velocities = rospy.get_param(
            "/guidance/dp/acceptance_velocities", [0.01, 0.01, 0.01, 0.01, 0.01, 0.001]
        )

        current_pose = odom_msg.pose.pose

        current_velocity_list = [
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z,
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z
        ]

        # create quats from msg
        goal_quat_list = [
            setpoint.orientation.x,
            setpoint.orientation.y,
            setpoint.orientation.z,
            -setpoint.orientation.w,  # invert goal quat
        ]
        current_quat_list = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ]
        q_r = quaternion_multiply(current_quat_list, goal_quat_list)

        # convert relative quat to euler
        (roll_diff, pitch_diff, yaw_diff) = euler_from_quaternion(q_r)

        # check if close to goal
        diff_list = [
            abs(setpoint.position.x - current_pose.position.x),
            abs(setpoint.position.y - current_pose.position.y),
            abs(setpoint.position.z - current_pose.position.z),
            roll_diff,
            pitch_diff,
            yaw_diff,
        ]
        is_close = True
        for i in range(len(diff_list)):
            if diff_list[i] > acceptance_margins[i]: 
                is_close = False
            elif current_velocity_list[i] > acceptance_velocities[i]:
                is_close = False

        return is_close
