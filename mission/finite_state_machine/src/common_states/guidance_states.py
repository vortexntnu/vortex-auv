#!/usr/bin/env python

import rospy
from smach import StateMachine, Sequence, Concurrence, cb_interface, CBState
from smach_ros import SimpleActionState
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal

from vortex_msgs.msg import (
    SetVelocityAction,
    SetVelocityActionGoal,
    LosPathFollowingAction,
    LosPathFollowingActionGoal,
)


def simple_dp_state(pose, action_server="/guidance_interface/dp_server"):
    """Create a SimpleActionState that travels to a goal pose using our DP guidance. Only use when in close
    proximity of goal pose.

    Args:
        pose (geometry_msgs/Pose): Goal pose.
        action_server (str, optional): name of dp action server. Defaults to "/guidance_interface/dp_server".

    Returns:
        SimpleActionState: state that travels to pose using DP guidance.
    """
    goal = MoveBaseActionGoal()
    goal.target_pose.pose = pose

    return SimpleActionState(action_server, MoveBaseAction, goal)


def simple_los_state(
    goal_positon,
    start_position,
    travel_speed=0.5,
    sphere_of_acceptance=0.5,
    action_server="/guidance_interface/los_server",
):
    """Creates a SimpleActionState for traveling to a goal position using LOS guidance.

    Args:
        goal_positon (geometry_msgs/Point): position drone should travel to.
        start_position (geometry_msgs/Point): position drone starts in.
        travel_speed (float, optional): forward velocity drone shoudl travel at. Defaults to 0.5.
        sphere_of_acceptance (float, optional): action returns success when drone is inside this sphere. Defaults to 0.5.
        action_server (str, optional): name of los action server. Defaults to "/guidance_interface/los_server".

    Returns:
        SimpleActionState: state that travel to from start to goal using LOS guidance
    """
    goal = LosPathFollowingActionGoal()
    goal.next_waypoint = goal_positon
    goal.prev_waypoint = start_position
    goal.forward_speed = travel_speed
    goal.desired_depth = goal_positon.z
    goal.sphereOfAcceptance = sphere_of_acceptance

    return SimpleActionState(action_server, LosPathFollowingAction, goal)


def simple_vel_state(twist, action_server="/guidance_interface/vel_server"):
    """Creates a SimpleActionState that sets drone velocity to a given twist.

    Args:
        twist (geometry_msgs/Twist): desired velocity
        action_server (str, optional): name of vel action server. Defaults to "/guidance_interface/vel_server".

    Returns:
        SimpleActionState: state that sets drone velocity to a given twist.
    """
    goal = SetVelocityActionGoal()
    goal.desired_velocity = twist

    return SimpleActionState(action_server, SetVelocityAction, goal)


def go_to(goal_pose, current_position):
    """Moves to a goal pose by first using LOS guindace for traveling potentially long distances
    and then DP guidance for fine tuned positioning.

    Args:
        goal_pose (geometry_msgs/Pose): Pose the drone will travel to. 
        current_position (geometry_msgs/Point): Position the drone starts in. This is needed for LOS guidance.

    Returns:
        Sequence: A sequence of a los action state and a dp action state
    """
    los_state = simple_los_state(goal_pose.position, current_position)
    dp_state = simple_dp_state(goal_pose)

    return create_sequence(
        [los_state, dp_state],
        state_names=["los_action_state", "dp_action_state"]
    )


def create_sequence(list_of_states, connector_outcome="succeeded", state_names=[]):
    """Creates a Sequence (container type) that connects the provided states. The first state
    in the list will also be the first state in the sequence, and so on. 
    All states in list_of_states must have the outcomes ["preempted", "succeeded", "aborted"].

    Args:
        list_of_states (list[State]): states that should be connected in a sequence
        connector_outcome (str, optional): The outcome that causes a transition to the next state. Defaults to "succeeded".
        state_names (list, optional): names that the states in the sequence are given. Defaults to [].

    Returns:
        Sequence: a Sequence of connected states
    """
    container = Sequence(
        outcomes=["preempted", "succeeded", "aborted"], connector_outcome=connector_outcome
    )
    with container:
        if len(list_of_states) == len(state_names):
            for name, state in zip(state_names, list_of_states):
                container.add(name, state)
        else:  # no state names provided
            counter = 0
            for state in list_of_states:
                container.add("State-%d" % counter, state)
                counter += 1
    return container
