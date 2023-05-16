#!/usr/bin/python3
# coding: UTF-8

# TODO: make into python module

from enum import IntEnum

import rospy
from smach_ros import SimpleActionState
from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import math
from tf.transformations import quaternion_from_euler
from scipy.spatial.transform import Rotation as R
import numpy as np

# Get action server parameters
guidance_interface_dp_action_server = rospy.get_param("/guidance/dp/action_server")
guidance_interface_los_action_server = rospy.get_param("/guidance/LOS/action_server")

# Enumeration for different control modes
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

def dp_move(x, y, z=-0.5, yaw_rad=0):
    """ Function to create a move goal with the specified parameters and returns a SimpleActionState
    Args:
        x, y, z: Coordinates for the move goal
        yaw_rad: Orientation of the goal in radians
    Returns:
        SimpleActionState with the goal
    """
    goal = MoveBaseGoal()
    goal.target_pose.pose.position = Point(x, y, z)
    goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw_rad))

    return SimpleActionState(guidance_interface_dp_action_server, MoveBaseAction, goal=goal)

def rotate_certain_angle(pose, angle):
    """ Function to rotate a pose by a certain angle
    Args:
        pose: Original pose
        angle: Angle to rotate by in degrees
    Returns:
        New pose after rotation
    """
    orientation = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    rotation = R.from_rotvec(angle * math.pi / 180 * np.array([0, 0, 1]))
    new_orientation = R.as_quat(orientation * rotation)
    new_pose = Pose()
    new_pose.position = pose.position
    new_pose.orientation.x = new_orientation[0]
    new_pose.orientation.y = new_orientation[1]
    new_pose.orientation.z = new_orientation[2]
    new_pose.orientation.w = new_orientation[3]

    return new_pose

def get_pose_in_front(pose, distance, index=0):
    """ Function to get a pose that is a certain distance in front of a given pose
    Args:
        pose: The original pose
        distance: Distance in meters to the desired pose
        index: Index for rotation matrix column selection
    Returns:
        The new pose
    """
    orientation_object = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    rotation_matrix = orientation_object.as_matrix()
    x_vec = rotation_matrix[:, index]
    current_pos_vec = np.array([pose.position.x, pose.position.y, pose.position.z])
    new_pose_vec = current_pos_vec + distance * x_vec

    new_pose = Pose()
    new_pose.position.x = new_pose_vec[0]
    new_pose.position.y = new_pose_vec[1]
    new_pose.position.z = new_pose_vec[2]
    new_pose.orientation = pose.orientation

    return new_pose

def get_position_on_line(from_pos, to_pos, distance):
    """ Function to get a position on a line a certain distance from the from_pos
    Args:
        from_pos: Starting position
        to_pos: End position
        distance: Distance from the starting position
    Returns:
        New position
    """
    p = Point()
    p.x = to_pos.x - from_pos.x
    p.y = to_pos.y - from_pos.y
    p.z = to_pos.z - from_pos.z

    length = math.sqrt(p.x**2 + p.y**2 + p.z**2)

    p.x = distance / length * p.x + from_pos.x
    p.y = distance / length * p.y + from_pos.y
    p.z = distance / length * p.z + from_pos.z

    return p

def get_pose_to_side(pose, unsigned_distance, chosen_side):
    """ Function to get a pose that is a certain distance to the side of a given pose
    Args:
        pose: The original pose
        unsigned_distance: Distance in meters to the desired pose
        chosen_side: True for left, False for right
    Returns:
        The new pose
    """
    orientation_object = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    rotation_matrix = orientation_object.as_matrix()
    y_vec = rotation_matrix[:, 1]
    current_pos_vec = np.array([pose.position.x, pose.position.y, pose.position.z])
    if chosen_side == True:
        new_pose_vec = current_pos_vec + unsigned_distance * y_vec
    else:
        new_pose_vec = current_pos_vec - unsigned_distance * y_vec

    new_pose = Pose()
    new_pose.position.x = new_pose_vec[0]
    new_pose.position.y = new_pose_vec[1]
    new_pose.position.z = new_pose_vec[2]
    new_pose.orientation = pose.orientation

    return new_pose
