#!/usr/bin/python3
# coding: UTF-8

from enum import IntEnum
from turtle import position

import rospy
from smach import StateMachine, Sequence, Concurrence, cb_interface, CBState
from smach_ros import SimpleActionState
from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal

import math
from vortex_msgs.srv import ControlMode
import actionlib
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)
from scipy.spatial.transform import Rotation as R
import numpy as np

guidance_interface_dp_action_server = rospy.get_param(
    "/guidance/dp/action_server")
guidance_interface_los_action_server = rospy.get_param(
    "/guidance/LOS/action_server")


# DP control modes
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

    goal = MoveBaseGoal()
    goal.target_pose.pose.position = Point(x, y, z)
    goal.target_pose.pose.orientation = Quaternion(
        *quaternion_from_euler(0, 0, yaw_rad))

    return SimpleActionState(guidance_interface_dp_action_server,
                             MoveBaseAction,
                             goal=goal)


def rotate_certain_angle(pose, angle):
    """Angle in degrees"""

    orientation = R.from_quat([
        pose.orientation.x, pose.orientation.y, pose.orientation.z,
        pose.orientation.w
    ])
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
    # returns pose that is distance meters in front of object pose

    orientation_object = R.from_quat([
        pose.orientation.x, pose.orientation.y, pose.orientation.z,
        pose.orientation.w
    ])
    rotation_matrix = orientation_object.as_matrix()
    x_vec = rotation_matrix[:, index]
    current_pos_vec = np.array(
        [pose.position.x, pose.position.y, pose.position.z])
    new_pose_vec = current_pos_vec + distance * x_vec

    new_pose = Pose()
    new_pose.position.x = new_pose_vec[0]
    new_pose.position.y = new_pose_vec[1]
    new_pose.position.z = new_pose_vec[2]
    new_pose.orientation = pose.orientation

    return new_pose


def get_position_on_line(from_pos, to_pos, distance):
    # returns pose that is distance meters in front of object, along line
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
    # returns pose that is distance meters to one side of the gate
    # chosen_side is either Bootlegger or G-man
    # Bootlegger (False) = right
    # G-man (True) = left

    orientation_object = R.from_quat([
        pose.orientation.x, pose.orientation.y, pose.orientation.z,
        pose.orientation.w
    ])
    rotation_matrix = orientation_object.as_matrix()
    y_vec = rotation_matrix[:, 1]
    current_pos_vec = np.array(
        [pose.position.x, pose.position.y, pose.position.z])
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


def patrol_sequence(action_states):

    sm = Sequence(outcomes=["preempted", "succeeded", "aborted"],
                  connector_outcome="succeeded")
    counter = 0

    with sm:

        for state in action_states:
            counter = counter + 1
            sm.add("State-%d" % counter, state)

    return sm


@cb_interface(
    outcomes=["alligned", "wrong_direction"],
    input_keys=["alligned_pose"],
    output_keys=["alligned_pose"],
)
def allignment_checker(userdata):
    # TODO
    pass


def allign_with_target(target):
    """
    Returns a state that alligns with the specified target.

    target: string on the form 'GATE' or 'BOUY'

    The returned state is responsible for chaning the circeling direction when needed.

    """

    # TODO: get target position (x,y) from landmark server

    # TODO: create a move_goal
    move_goal = None

    allignment_attempt = Concurrence(
        outcomes=["succeeded", "preempted", "wrong_direction"],
        outcome_map={
            "succeeded": {
                "ALLIGNMENT_CHECKER": "alligned"
            },
            "wrong_direction": {
                "ALLIGNMENT_CHECKER": "wrong_direction"
            },
        },
        default_outcome=["preempted"],
        child_termination_cb=None,  # TODO: should allways terminate
    )

    with allignment_attempt:

        Concurrence.add(
            "CIRCLE_GATE",
            SimpleActionState("controller/move", MoveAction,
                              goal=move_goal),  # TODO
        )
        Concurrence.add("ALLIGNMENT_CHECKER", CBState(allignment_checker))


def create_circle_coordinates(start, centre, angle, counterclockwise=True):
    resolution = 0.1  # distance between points
    coordinates = []
    radius = math.sqrt(
        abs((start.x - centre.x)**2) + abs((centre.y - start.y)**2))
    circumference = 2 * radius * math.pi
    number_of_points = int(math.ceil(circumference / resolution))
    angle_to_add = angle / number_of_points
    x = start.x - centre.x
    y = start.y - centre.y
    start_angle = math.atan2(y, x) * 180 / math.pi
    if not counterclockwise:
        start_angle -= angle
    for i in range(number_of_points + 1):
        coordX = centre.x + radius * math.cos(start_angle * math.pi / 180)
        coordY = centre.y + radius * math.sin(start_angle * math.pi / 180)
        coordinates.append(Point(coordX, coordY, centre.z))
        start_angle += angle_to_add
    if not counterclockwise:
        return coordinates[::-1]
    else:
        return coordinates


def within_acceptance_margins(setpoint, odom_msg, check_yaw_only=False):
    if check_yaw_only:
        acceptance_margins = [100, 100, 100, 100, 100, 0.1]
        acceptance_velocities = [100, 100, 100, 100, 100, 100]

    else:
        acceptance_margins = rospy.get_param("/guidance/dp/acceptance_margins",
                                             [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        acceptance_velocities = rospy.get_param(
            "/guidance/dp/acceptance_velocities",
            [0.01, 0.01, 0.01, 0.01, 0.01, 0.001])

    current_pose = odom_msg.pose.pose

    current_velocity_list = [
        odom_msg.twist.twist.linear.x,
        odom_msg.twist.twist.linear.y,
        odom_msg.twist.twist.linear.z,
        odom_msg.twist.twist.angular.x,
        odom_msg.twist.twist.angular.y,
        odom_msg.twist.twist.angular.z,
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
        abs(roll_diff),
        abs(pitch_diff),
        abs(yaw_diff),
    ]
    is_close = True
    for i in range(len(diff_list)):
        if diff_list[i] > acceptance_margins[i]:
            is_close = False
        elif current_velocity_list[i] > acceptance_velocities[i]:
            is_close = False

    return is_close
