#!/usr/bin/env python

import rospy
from smach import StateMachine, Sequence, Concurrence, cb_interface, CBState, State
from smach_ros import SimpleActionState
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from vortex_msgs.msg import (
    SetVelocityAction,
    SetVelocityGoal,
    LosPathFollowingAction,
    LosPathFollowingGoal,
)
from helper import create_sequence


class GoToState(State):
    def __init__(self, goal_pose):
        """State that moves to a goal pose by first using LOS guindace for traveling
        potentially long distances and then DP guidance for fine tuned positioning.

        Args:
            goal_pose (geometry_msgs/Pose): Pose the drone will travel to.
        """
        super().__init__(outcomes=["preempted", "succeeded", "aborted"])
        self.goal_pose = goal_pose

    def execute(self, ud):
        los_state = los_state(self.goal_pose.position)
        dp_state = dp_state(self.goal_pose)

        sm = create_sequence(
            [los_state, dp_state], state_names=["los_move_state", "dp_move_state"]
        )
        sm.execute()


def dp_state(pose, action_server="/guidance_interface/dp_server"):
    """Create a SimpleActionState that travels to a goal pose using our DP guidance.
    Only use when in close proximity of goal pose.

    Args:
        pose (geometry_msgs/Pose): Goal pose.
        action_server (str, optional):
                name of dp action server. Defaults to "/guidance_interface/dp_server".

    Returns:
        SimpleActionState: state that travels to pose using DP guidance.
    """
    goal = MoveBaseGoal()
    goal.target_pose.pose = pose

    return SimpleActionState(action_server, MoveBaseAction, goal)


def los_state(
    goal_positon,
    travel_speed=0.5,
    sphere_of_acceptance=0.5,
    action_server="/guidance_interface/los_server",
):
    """Creates a SimpleActionState for traveling to a goal position using LOS guidance.

    Args:
        goal_positon (geometry_msgs/Point): position drone should travel to.
        start_position (geometry_msgs/Point): position drone starts in.
        travel_speed (float, optional):
                forward velocity drone shoudl travel at. Defaults to 0.5.
        sphere_of_acceptance (float, optional):
                action returns success when drone is inside this sphere. Defaults to 0.5.
        action_server (str, optional):
                name of los action server. Defaults to "/guidance_interface/los_server".

    Returns:
        SimpleActionState: state that travel to from start to goal using LOS guidance
    """
    goal = LosPathFollowingGoal()
    goal.next_waypoint = goal_positon
    goal.forward_speed = travel_speed
    goal.desired_depth = goal_positon.z
    goal.sphereOfAcceptance = sphere_of_acceptance

    return SimpleActionState(action_server, LosPathFollowingAction, goal)


def vel_state(twist, action_server="/guidance_interface/vel_server"):
    """Creates a SimpleActionState that sets drone velocity to a given twist.

    Args:
        twist (geometry_msgs/Twist): desired velocity
        action_server (str, optional):
                name of vel action server. Defaults to "/guidance_interface/vel_server".

    Returns:
        SimpleActionState: state that sets drone velocity to a given twist.
    """
    goal = SetVelocityGoal()
    goal.desired_velocity = twist

    return SimpleActionState(action_server, SetVelocityAction, goal)
