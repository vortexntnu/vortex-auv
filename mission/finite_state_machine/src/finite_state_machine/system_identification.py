#!/usr/bin/env python

import rospy
from smach import State
from smach_ros import IntrospectionServer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import euler_to_quaternion, quaternion_to_euler, Quaternion, quaternion_multiply

from common_states import GoToState, vel_state
from helper import create_sequence, point, pose, twist


class Monitor(State):
    def __init__(self, goal_pose, max_duration, pool_bounds, goal_boundry, odom_topic="/odometry/filtered"):
        """State that monitors in drone is within pool bounds, close enough to goal or a timeout has occured.

        Args:
            goal_pose (geometry_msgs/Pose): the pose the drone aims for
            max_duration (double): duration before timeout occurs
            pool_bounds (list[Tuple]): list with tuples of (x_min, x_max) for x, y, and z
            goal_boundry (list[double]): boundries for how close drone must be to goal pose in 6DOF
            odom_topic (str, optional): Topic for odometry. Defaults to "/odometry/filtered".
        """
        super().__init__(self, outcomes=["preempted", "succeeded", "aborted"])
        self.duration = max_duration
        self.timeout = False

        self.goal_pose = goal_pose
        self.odom = Odometry()
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.update_odom)

        self.x_min, self.x_max = self.pool_bounds[0]
        self.y_min, self.y_max = self.pool_bounds[1]
        self.z_min, self.z_max = self.pool_bounds[2]

        # create quats from msg
        goal_orientation_list = [
            self.goal_pose.pose.orientation.x, 
            self.goal_pose.pose.orientation.y, 
            self.goal_pose.pose.orientation.z, 
            self.goal_pose.pose.orientation.w  
        ]
        self.goal_quat = Quaternion(goal_orientation_list)
        self.current_quat = Quaternion()

    def execute(self, ud):
        # start timer
        rospy.Timer(rospy.Duration(self.duration), self.timer_cb, oneshot=True)

        while not rospy.is_shutdown() and not timeout:

            if not self.within_bounds():
                return 'succeeded'

            if self.close_to_goal():
                return 'succeeded'

        return 'succeeded'

    def update_odom(self, odom_msg):
        self.odom = odom_msg
        current_orientation_list = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        ]
        self.current_quat = Quaternion(current_orientation_list)

    def timer_cb(self, event):
        self.timeout = True

    def within_bounds(self):
        if not (x_min <= self.odom.pose.pose.position.x <= x_max):
            return False
        if not (y_min <= self.odom.pose.pose.position.y <= y_max):
            return False
        if not (z_min <= self.odom.pose.pose.position.z <= z_max):
            return False
        return True

    def close_to_goal(self):
        
        # find relative quat
        q_r = quaternion_multiply(self.current_quat, self.goal_quat.inverse())

        # convert relative quat to euler 
        (roll_diff, pitch_diff, yaw_diff) = quaternion_to_euler(q_r)

        # check if close to goal
        diff_list = [
            abs(self.goal_pose.position.x - self.odom.pose.pose.position.x), 
            abs(self.goal_pose.position.y - self.odom.pose.pose.position.y),
            abs(self.goal_pose.position.z - self.odom.pose.pose.position.z),
            roll_diff,
            pitch_diff,
            yaw_diff
        ]
        is_close = True
        for diff, bound in zip(diff_list, self.goal_boundry):
            if diff > boud:
                is_close = False

        return is_close


class SingleTest(State):
    def __init__(self, twist, start_pose, goal_pose, timeout=10, goal_boundry=[0.5, 0.5, 0.2, 0.15, 0.15, 0.15]):
        super().__init__(outcomes=["preempted", "succeeded", "aborted"])
        self.twist = twist
        self.goal_pose = goal_pose
        self.start_pose = start_pose
        self.timeout = timeout
        self.goal_boundry = goal_boundry

        self.x_min = 0
        self.x_max = 10        
        self.y_min = -2
        self.y_max = 2
        self.z_min = -1.2
        self.z_max = -0.3

    def execute(self, ud):
        states = [
            GoToState(self.start_pose),
            vel_state(self.twist),
            Monitor(
                goal_pose=self.goal_pose, 
                duration=self.timeout, 
                pool_bounds=[(self.x_min, self.x_max), (self.y_min, self.y_max), (self.z_min, self.z_max)], 
                goal_boundry=goal_boundry,
                odom_topic="/odometry/filtered"
            ),
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
        )
    ]
    sm = create_sequence(states)
    introspection_server = IntrospectionServer(str(rospy.get_name()), sm,'/SM_ROOT')

    introspection_server.start()
    sm.execute()


if __name__ == "__main__":
    surge_sway_heave()
