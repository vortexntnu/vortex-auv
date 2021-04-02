#!/usr/bin/env python

import rospy
from smach import State
from nav_msgs.msg import Odometry, Twist

from common_states import GoToState, vel_state
from helper import create_sequence, point, pose, twist


class Monitor(State):
    def __init__(self, goal_pose, timeout, odom_topic="/odometry/filtered"):
        super().__init__(self, outcomes=["preempted", "succeeded", "aborted"])
        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.update_odom)

    def execute(self, ud):
        # start timer

        while not rospy.is_shutdown():
            # check if within bounds

            # check if within range of acceptance

            # check if timeout

            pass
        return "preempted"

    def update_odom(self, odom_msg):
        self.odom = odom_msg


class SingleTest(State):
    def __init__(self, twist, start_pose, goal_pose, timeout=10):
        super().__init__(outcomes=["preempted", "succeeded", "aborted"])
        self.twist = twist
        self.goal_pose = goal_pose
        self.start_pose = start_pose
        self.timeout = timeout

    def execute(self, ud):
        states = [
            GoToState(self.start_pose),
            vel_state(self.twist),
            Monitor(self.goal_pose, self.timeout),
            GoToState(self.start_pose),
        ]
        names = ["go_to_start", "set_velocity", "monitor", "back_to_start"]
        sm = create_sequence(states, state_names=names)
        sm.execute()


def surge_sway_heave():
    states = [
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
        SingleTest(
            twist(1, 0, 0, 0, 0, 0), pose(0, 0, 0, 0, 0, 0), pose(5, 0, 0, 0, 0, 0)
        ),
    ]
    sm = create_sequence(states)
    sm.execute()


if __name__ == "__main__":
    surge_sway_heave()
