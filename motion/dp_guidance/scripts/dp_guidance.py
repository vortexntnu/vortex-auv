#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.


from enum import IntEnum

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseActionGoal
from geometry_msgs.msg import Pose
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


class DPGuidance:
    """
    Take an input goal from an action client and pass
    it on to the reference model, but this time on a topic.

    When a goal is given to this module, it will begin to publish
    data for the reference model, which in turn makes the dp
    controller produce a thrust vector. It will only stop
    publishing once the goal has been preempted.
    """

    def __init__(self):
        """
        Create the ROS node dp and set constants, as well as the action
        server that the fsm connects to. Connect to the move_base action
        server in the dp controller. The guidance and controller communicate
        through this server.
        """

        # params
        rate = rospy.get_param("guidance/dp/rate", 20)  # [Hz]
        self.acceptance_margins = rospy.get_param(
            "/guidance/dp/acceptance_margins", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        )
        self.action_server_max_duration = rospy.get_param(
            "/guidance/dp/max_duration", default=60
        )

        # init internal variables
        self.ros_rate = rospy.Rate(rate)
        self.current_pose = Pose()
        self.controller_setpoint = Pose()
        self.control_mode = ControlModeEnum.OPEN_LOOP.value
        self.dp_timout = False

        # Publisher for the reference model
        self.reference_model_pub = rospy.Publisher(
            "/guidance/dp_data", DpSetpoint, queue_size=1
        )

        # Subscriber for state (for acceptance margins)
        self.state_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.update_current_pose
        )

        # Service for chaning control mode
        self.control_mode_service = rospy.Service(
            "dp_guidance/controlmode_service", ControlMode, self.control_mode_service_cb
        )

        # Action server for receiving goal data
        rospy.logdebug("Starting dp_guidance action server..")
        self.action_server = actionlib.SimpleActionServer(
            name="dp_action_server",
            ActionSpec=MoveBaseAction,
            execute_cb=self.action_cb,
            auto_start=False,
        )
        self.action_server.start()
        rospy.logdebug("DP guidance initialized")

    def action_cb(self, goal):

        # set new setpoint and control mode
        self.controller_setpoint =  goal.target_pose.pose
        self.control_mode = ControlModeEnum.POSE_HOLD.value
        rospy.logdebug(
            "DP has recieved new goal with x: %d, y: %d, z: %d"
            % (
                self.controller_setpoint.position.x,
                self.controller_setpoint.position.y,
                self.controller_setpoint.position.z,
            )
        )

        # start a timout Timer
        self.dp_timout = False
        timer = rospy.Timer(
            rospy.Duration(self.action_server_max_duration),
            self.set_timeout,
            oneshot=True,
        )

        # wait for goal to be reached
        check_rate = rospy.Rate(2)
        while not rospy.is_shutdown():

            # check for preempt request
            if self.action_server.is_preempt_requested():
                self.control_mode = ControlModeEnum.OPEN_LOOP.value
                self.action_server.set_preempted()
                break

            # check if goal is reached
            if self.within_acceptance_margins():
                res = MoveBaseActionResult()
                self.action_server.set_succeeded(result=res)
                break

            # check if timeout is reached
            if self.dp_timout:
                self.action_server.set_aborted()
                rospy.logwarn("DP guidance aborted action due to timeout")
                break

            check_rate.sleep()
            
        timer.shutdown()

    def control_mode_service_cb(self, control_mode_req):
        # change control mode
        self.control_mode = control_mode_req.controlmode

        # update setpoint to current point
        self.controller_setpoint = self.current_pose

        return []  # empty list for no result

    def update_current_pose(self, odom_msg):
        self.current_pose = odom_msg.pose.pose

    def set_timeout(self, event):
        self.dp_timout = True

    def within_acceptance_margins(self):
        # create quats from msg
        goal_quat_list = [
            self.controller_setpoint.orientation.x,
            self.controller_setpoint.orientation.y,
            self.controller_setpoint.orientation.z,
            -self.controller_setpoint.orientation.w,  # invert goal quat
        ]
        current_quat_list = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ]
        q_r = quaternion_multiply(current_quat_list, goal_quat_list)

        # convert relative quat to euler
        (roll_diff, pitch_diff, yaw_diff) = euler_from_quaternion(q_r)

        # check if close to goal
        diff_list = [
            abs(self.controller_setpoint.position.x - self.current_pose.position.x),
            abs(self.controller_setpoint.position.y - self.current_pose.position.y),
            abs(self.controller_setpoint.position.z - self.current_pose.position.z),
            roll_diff,
            pitch_diff,
            yaw_diff,
        ]

        is_close = True
        for i in range(len(diff_list)):
            if diff_list[i] > self.acceptance_margins[i]:
                is_close = False
        return is_close

    def spin(self):
        """
        A replacement for the normal rospy.spin(), equivalent
        to "spinOnce" in roscpp (except simpler, since each cb/topic
        has it's own thread)
        """
        dp_setpoint = DpSetpoint()
        while not rospy.is_shutdown():

            # publish setpoint and sleep
            dp_setpoint.setpoint = self.controller_setpoint
            dp_setpoint.control_mode = self.control_mode
            self.reference_model_pub.publish(dp_setpoint)
            self.ros_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("dp_guidance")
    dp_guidance = DPGuidance()
    dp_guidance.spin()
