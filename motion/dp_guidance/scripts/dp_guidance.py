#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.


from enum import IntEnum

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
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
            "guidance/dp/acceptance_margins", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        )

        # init internal variables
        self.ros_rate = rospy.Rate(rate)
        self.current_pose = Pose()
        self.controller_setpoint = Pose()
        self.control_mode = ControlModeEnum.OPEN_LOOP.value

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
        self.action_server = actionlib.SimpleActionServer(
            name="dp_action_server", ActionSpec=MoveBaseAction, auto_start=False
        )
        self.action_server.register_goal_callback(
            self.goal_cb
        )  # Called whenever guidance_interface sends a new goal for the dp system
        self.action_server.register_preempt_callback(self.preempt_cb)
        self.action_server.start()

    def control_mode_service_cb(self, control_mode_req):
        # cancel any active actions
        if self.action_server.current_goal.goal:
            self.action_server.current_goal.set_canceled()
            self.action_server.set_preempted()
            rospy.logdebug(
                "dp_guidance action server preempted from control mode change"
            )

        # change control mode
        self.control_mode = control_mode_req.controlmode

        # update setpoint to current point
        self.controller_setpoint = self.current_pose
        
        return []  # empty list for no result

    def update_current_pose(self, odom_msg):
        self.current_pose = odom_msg.pose.pose

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

    def goal_cb(self):
        """
        Accept a goal from the guidance interface and store it as local state,
        then activate publipublish_guidance_datashing
        """
        new_goal = self.action_server.accept_new_goal()
        self.controller_setpoint = new_goal.target_pose.pose
        rospy.logdebug(
            "DP has recieved new goal with x: %d, y: %d, z: %d"
            % (
                self.controller_setpoint.position.x,
                self.controller_setpoint.position.y,
                self.controller_setpoint.position.z,
            )
        )

    def preempt_cb(self):
        """
        The preempt callback for the action server.
        """
        if self.action_server.is_preempt_requested():
            self.action_server.current_goal.set_canceled()
            self.action_server.set_preempted()
            rospy.logdebug("Goal action server in dp_guidance was preempted!")

            self.control_mode = ControlModeEnum.OPEN_LOOP.value

    def spin(self):
        """
        A replacement for the normal rospy.spin(), equivalent
        to "spinOnce" in roscpp (except simpler, since each cb/topic
        has it's own thread)
        """
        while not rospy.is_shutdown():

            # if action server is active, check if within acceptance margins
            if self.action_server.current_goal.goal:
                if self.within_acceptance_margins():
                    self.action_server.current_goal.set_succeeded()
                    self.action_server.set_succeeded()

            # publish setpoint and sleep
            dp_setpoint = DpSetpoint()
            dp_setpoint.setpoint = self.controller_setpoint
            dp_setpoint.control_mode = self.control_mode
            self.reference_model_pub.publish(dp_setpoint)
            self.ros_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("dp_guidance", log_level=rospy.DEBUG)

    try:
        dp_guidance = DPGuidance()
        dp_guidance.spin()

    except rospy.ROSInternalException as e:
        rospy.logerr(e)
