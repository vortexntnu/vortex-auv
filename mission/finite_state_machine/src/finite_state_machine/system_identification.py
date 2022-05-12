#!/usr/bin/python3

import rospy
from smach import State
from smach_ros import IntrospectionServer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)

from common_states import GoToState, vel_state, VelState
from helper import create_sequence, point, pose, twist, ControlModeEnum


class Monitor(State):
    def __init__(
        self,
        max_duration,
        pool_bounds,
        goal_pose=None,
        goal_boundry=None,
        odom_topic="/odometry/filtered",
    ):
        """State that monitors in drone is within pool bounds, close enough to goal or a timeout has occured.

        Args:
            goal_pose (geometry_msgs/Pose): the pose the drone aims for
            max_duration (double): duration before timeout occurs
            pool_bounds (list[Tuple]): list with tuples of (x_min, x_max) for x, y, and z
            goal_boundry (list[double]): boundries for how close drone must be to goal pose in 6DOF
            odom_topic (str, optional): Topic for odometry. Defaults to "/odometry/filtered".
        """
        State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])
        self.rate = rospy.Rate(10)
        self.duration = max_duration
        self.timeout = False

        self.goal_pose = goal_pose
        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.update_odom)

        self.x_min, self.x_max = pool_bounds[0]
        self.y_min, self.y_max = pool_bounds[1]
        self.z_min, self.z_max = pool_bounds[2]

        self.goal_boundry = goal_boundry

    def execute(self, ud):
        # start timer
        self.timeout = False
        timer = rospy.Timer(rospy.Duration(self.duration), self.timer_cb, oneshot=True)

        while not rospy.is_shutdown():

            if not self.within_bounds():
                rospy.loginfo("[Monitor] Drone is out if bounds")
                return "succeeded"

            if self.close_to_goal():
                rospy.loginfo("[Monitor] Drone is within goal range")
                return "succeeded"

            if self.timeout:
                rospy.loginfo("[Monitor] Timout has occured")
                return "succeeded"

            self.rate.sleep()

        timer.shutdown()

        return "preempted"

    def update_odom(self, odom_msg):
        self.odom = odom_msg

    def timer_cb(self, event):
        self.timeout = True

    def within_bounds(self):
        if not (self.x_min <= self.odom.pose.pose.position.x <= self.x_max):
            return False
        if not (self.y_min <= self.odom.pose.pose.position.y <= self.y_max):
            return False
        if not (self.z_min <= self.odom.pose.pose.position.z <= self.z_max):
            return False
        return True

    def close_to_goal(self):
        # check if goal_pose in use
        if not self.goal_pose:
            return False

        # create quats from msg
        goal_quat_list = [
            self.goal_pose.orientation.x,
            self.goal_pose.orientation.y,
            self.goal_pose.orientation.z,
            -self.goal_pose.orientation.w,  # invert goal quat
        ]
        current_quat_list = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ]
        q_r = quaternion_multiply(current_quat_list, goal_quat_list)

        # convert relative quat to euler
        (roll_diff, pitch_diff, yaw_diff) = euler_from_quaternion(q_r)

        # check if close to goal
        diff_list = [
            abs(self.goal_pose.position.x - self.odom.pose.pose.position.x),
            abs(self.goal_pose.position.y - self.odom.pose.pose.position.y),
            abs(self.goal_pose.position.z - self.odom.pose.pose.position.z),
            roll_diff,
            pitch_diff,
            yaw_diff,
        ]
        is_close = True
        for diff, bound in zip(diff_list, self.goal_boundry):
            if diff > bound:
                is_close = False

        return is_close


class SingleTest(State):
    def __init__(
        self,
        twist,
        start_pose,
        dp_mode=ControlModeEnum.OPEN_LOOP,
        goal_pose=None,
        timeout=20,
        goal_boundry=[0.5, 0.5, 0.2, 0.15, 0.15, 0.15],
        end_pose=None,
    ):
        State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        x_min = -2
        x_max = 2
        y_min = -0.85
        y_max = 0.85
        z_min = 0.4
        z_max = 1.05

        states = [
            GoToState(start_pose),
            VelState(twist, dp_control_mode=dp_mode),
            Monitor(
                goal_pose=goal_pose,
                max_duration=timeout,
                pool_bounds=[
                    (x_min, x_max),
                    (y_min, y_max),
                    (z_min, z_max),
                ],
                goal_boundry=goal_boundry,
                odom_topic="/odometry/filtered",
            ),
        ]
        if not end_pose:
            states.append(GoToState(start_pose))
        else:
            states.append(GoToState(end_pose))

        names = ["go_to_start", "set_velocity", "monitor", "back_to_start"]
        self.sm = create_sequence(states, state_names=names)

    def execute(self, ud):
        return self.sm.execute()


def surge_tests():
    class SurgeTest(SingleTest):
        def __init__(self, velocity, orientation, end_pose=None):
            SingleTest.__init__(
                self,
                twist(X=velocity),
                pose(-1.5, 0, 0.7, 0, 0, orientation),
                dp_mode=ControlModeEnum.ORIENTATION_DEPTH_HOLD,
                end_pose=end_pose,
            )

    states = [
        SurgeTest(0.05, 0),
        SurgeTest(0.10, 0),
        SurgeTest(0.15, 0),
        SurgeTest(0.20, 0),
        SurgeTest(0.25, 0),
        SurgeTest(0.30, 0, end_pose=HOME_POSE),
        SurgeTest(-0.05, 180),
        SurgeTest(-0.10, 180),
        SurgeTest(-0.15, 180),
        SurgeTest(-0.20, 180),
        SurgeTest(-0.25, 180),
        SurgeTest(-0.30, 180, end_pose=HOME_POSE),
    ]
    return states


def sway_tests():
    class SwayTest(SingleTest):
        def __init__(self, velocity, orientation, end_pose=None):
            SingleTest.__init__(
                self,
                twist(Y=velocity),
                pose(-1.5, 0, 0.7, 0, 0, orientation),
                dp_mode=ControlModeEnum.ORIENTATION_DEPTH_HOLD,
                end_pose=end_pose,
            )

    states = [
        SwayTest(0.05, -90),
        SwayTest(0.10, -90),
        SwayTest(0.15, -90),
        SwayTest(0.20, -90),
        SwayTest(0.25, -90),
        SwayTest(0.30, -90, end_pose=HOME_POSE),
        SwayTest(-0.05, 90),
        SwayTest(-0.10, 90),
        SwayTest(-0.15, 90),
        SwayTest(-0.20, 90),
        SwayTest(-0.25, 90),
        SwayTest(-0.30, 90, end_pose=HOME_POSE),
    ]
    return states


def roll_tests():
    states = [
        SingleTest(
            twist(0.1, 0, 0, 0.1, 0, 0),
            pose(-2, 0, 0.7, 0, 0, 0),
            goal_pose=pose(0, 0, 0.7, 90, 0, 0),
            dp_mode=ControlModeEnum.POSITION_HEADING_HOLD,
        ),
    ]
    return states


def yaw_tests():
    class YawTest(SingleTest):
        def __init__(self, yaw_vel):
            SingleTest.__init__(
                self,
                twist(0.0, 0, 0, 0.0, 0, yaw_vel),
                pose(0, 0, 0.7, 0, 0, 0),
                timeout=15,
                dp_mode=ControlModeEnum.DEPTH_HOLD,
            )

    states = [
        YawTest(0.1),
        YawTest(-0.1),
        YawTest(0.15),
        YawTest(-0.15),
        YawTest(0.2),
        YawTest(-0.2),
        YawTest(0.25),
        YawTest(-0.25),
        YawTest(0.3),
        YawTest(-0.3),
        YawTest(0.4),
        YawTest(-0.4),
        YawTest(0.5),
        YawTest(-0.5),
        YawTest(0.6),
        YawTest(-0.6),
    ]

    return states


def surge_sway_tests():
    class SurgeSwayTest(SingleTest):
        def __init__(self, surge_vel, sway_vel, initial_angle, end_pose=None):
            SingleTest.__init__(
                self,
                twist(surge_vel, sway_vel, 0, 0, 0, 0),
                pose(-1.5, 0, 0.7, 0, 0, initial_angle),
                timeout=20,
                dp_mode=ControlModeEnum.ORIENTATION_DEPTH_HOLD,
                end_pose=end_pose,
            )

    states = [
        SurgeSwayTest(0.05, 0.05, -45),
        SurgeSwayTest(0.10, 0.10, -45),
        SurgeSwayTest(0.15, 0.15, -45),
        SurgeSwayTest(0.20, 0.20, -45),
        SurgeSwayTest(0.25, 0.25, -45),
        SurgeSwayTest(0.30, 0.30, -45, end_pose=HOME_POSE),
        SurgeSwayTest(0.05, -0.05, 45),
        SurgeSwayTest(0.10, -0.10, 45),
        SurgeSwayTest(0.15, -0.15, 45),
        SurgeSwayTest(0.20, -0.20, 45),
        SurgeSwayTest(0.25, -0.25, 45),
        SurgeSwayTest(0.30, -0.30, 45, end_pose=HOME_POSE),
        SurgeSwayTest(-0.05, 0.05, -135),
        SurgeSwayTest(-0.10, 0.10, -135),
        SurgeSwayTest(-0.15, 0.15, -135),
        SurgeSwayTest(-0.20, 0.20, -135),
        SurgeSwayTest(-0.25, 0.25, -135),
        SurgeSwayTest(-0.30, 0.30, -135, end_pose=HOME_POSE),
        SurgeSwayTest(-0.05, -0.05, 135),
        SurgeSwayTest(-0.10, -0.10, 135),
        SurgeSwayTest(-0.15, -0.15, 135),
        SurgeSwayTest(-0.20, -0.20, 135),
        SurgeSwayTest(-0.25, -0.25, 135),
        SurgeSwayTest(-0.30, -0.30, 135),
    ]

    return states


def heave_tests():
    class HeaveTest(SingleTest):
        def __init__(self, velocity, start_depth, end_pose=None):
            SingleTest.__init__(
                self,
                twist(0, 0, velocity, 0, 0, 0),
                pose(0, 0, start_depth, 0, 0, 0),
                dp_mode=ControlModeEnum.ORIENTATION_HOLD,
                end_pose=end_pose,
            )

    states = [
        HeaveTest(0.05, 0.5),
        HeaveTest(0.10, 0.5),
        HeaveTest(0.15, 0.5),
        HeaveTest(-0.05, 1),
        HeaveTest(-0.10, 1),
        HeaveTest(-0.15, 1),
    ]
    return states


def heave_surge_tests():
    class HeaveSurgeTest(SingleTest):
        def __init__(
            self, heave_vel, surge_vel, initial_angle, initial_depth, end_pose=None
        ):
            SingleTest.__init__(
                self,
                twist(surge_vel, 0, heave_vel, 0, 0, 0),
                pose(-1.5, 0, initial_depth, 0, 0, initial_angle),
                dp_mode=ControlModeEnum.ORIENTATION_HOLD,
                end_pose=end_pose,
            )

    states = [
        HeaveSurgeTest(0.05, 0.1, 0, 0.5),
        HeaveSurgeTest(0.05, 0.2, 0, 0.5),
        HeaveSurgeTest(0.1, 0.1, 0, 0.5),
        HeaveSurgeTest(0.1, 0.2, 0, 0.5),
        HeaveSurgeTest(0.15, 0.1, 0, 0.5),
        HeaveSurgeTest(0.15, 0.2, 0, 0.5, end_pose=HOME_POSE),
        HeaveSurgeTest(-0.05, 0.1, 0, 1.0),
        HeaveSurgeTest(-0.05, 0.2, 0, 1.0),
        HeaveSurgeTest(-0.1, 0.1, 0, 1.0),
        HeaveSurgeTest(-0.1, 0.2, 0, 1.0),
        HeaveSurgeTest(-0.15, 0.1, 0, 1.0),
        HeaveSurgeTest(-0.15, 0.2, 0, 1.0, end_pose=HOME_POSE),
        HeaveSurgeTest(0.05, -0.1, 180, 0.5),
        HeaveSurgeTest(0.05, -0.2, 180, 0.5),
        HeaveSurgeTest(0.10, -0.1, 180, 0.5),
        HeaveSurgeTest(0.10, -0.2, 180, 0.5),
        HeaveSurgeTest(0.15, -0.1, 180, 0.5),
        HeaveSurgeTest(0.15, -0.2, 180, 0.5, end_pose=HOME_POSE),
        HeaveSurgeTest(-0.05, -0.1, 180, 1.0),
        HeaveSurgeTest(-0.05, -0.2, 180, 1.0),
        HeaveSurgeTest(-0.10, -0.1, 180, 1.0),
        HeaveSurgeTest(-0.10, -0.2, 180, 1.0),
        HeaveSurgeTest(-0.15, -0.1, 180, 1.0),
        HeaveSurgeTest(-0.15, -0.2, 180, 1.0),
    ]
    return states


if __name__ == "__main__":
    rospy.init_node("system_identification_sm")

    HOME_POSE = pose(0, 0, 0.7, 0, 0, 0)
    states = yaw_tests()

    sm = create_sequence(states)
    sm.execute()
