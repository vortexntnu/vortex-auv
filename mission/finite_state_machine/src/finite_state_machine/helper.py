#!/usr/bin/env python

from smach import Sequence
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


def twist(X, Y, Z, K, M, N):
    new_twist = Twist()
    new_twist.linear.x = X
    new_twist.linear.y = Y
    new_twist.linear.z = Z
    new_twist.angular.x = K
    new_twist.angular.y = M
    new_twist.angular.z = N

    return new_twist


def pose(x, y, z, roll, pitch, yaw):
    """creates a geometry_msgs/Pose 

    Args:
        x (double): [description]
        y (double): [description]
        z (double): [description]
        roll (double): [description]
        pitch (double): [description]
        yaw (double): [description]

    Returns:
        geometry_msgs/Pose: a pose 
    """
    new_pose = Pose()
    new_pose.position.x = x
    new_pose.position.y = y
    new_pose.position.z = z
    new_pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))

    return new_pose


def point(x, y, z):
    new_point = Point()
    new_point.x = x
    new_point.y = y
    new_point.z = z

    return new_point


def create_sequence(list_of_states, connector_outcome="succeeded", state_names=[]):
    """Creates a Sequence (container type) that connects the provided states.
    The first state in the list will also be the first state in the sequence, and so on.
    All states in list_of_states must have the outcomes ["preempted", "succeeded", "aborted"].

    Args:
        list_of_states (list[State]): states that should be connected in a sequence
        connector_outcome (str, optional):
                The outcome that causes a transition to the next state. Defaults to "succeeded".
        state_names (list, optional):
                names that the states in the sequence are given. Defaults to [].

    Returns:
        Sequence: a Sequence of connected states
    """
    container = Sequence(
        outcomes=["preempted", "succeeded", "aborted"],
        connector_outcome=connector_outcome,
    )
    with container:
        if len(list_of_states) == len(state_names):
            for name, state in zip(state_names, list_of_states):
                Sequence.add(name, state)
        else:  # no state names provided
            counter = 0
            for state in list_of_states:
                Sequence.add("State-%d" % counter, state)
                counter += 1
    return container