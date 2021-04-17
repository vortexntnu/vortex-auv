#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.

import rospy
import actionlib

from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

from std_srvs.srv import SetBool


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

        rate = rospy.get_param("guidance/dp/rate", 20)  # [Hz]
        self.acceptance_margins = rospy.get_param("guidance/dp/acceptance_margins", [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        self.ros_rate = rospy.Rate(rate)
        self.publish_guidance_data = False
        self.controller_setpoint = Pose()
        self.current_pose = Pose()

        # Publisher for the reference model
        self.reference_model_pub = rospy.Publisher(
            "/guidance/dp_data", Pose, queue_size=1
        )

        # Subscriber for state (for acceptance margins)
        self.state_sub = rospy.Subscriber(
            "/odometry/filtered", Odometry , self.update_current_pose
        )

        # Action server for receiving goal data
        self.goal_action_server = actionlib.SimpleActionServer(
            name="dp_action_server", ActionSpec=MoveBaseAction, auto_start=False
        )
        self.goal_action_server.register_goal_callback(
            self.goal_cb
        )  # Called whenever guidance_interface sends a new goal for the dp system
        self.goal_action_server.register_preempt_callback(self.preempt_cb)
        self.goal_action_server.start()

    
    def update_current_pose(self, odom_msg):
        self.current_pose = odom_msg.pose.pose

    def within_acceptance_margins(self):
        # create quats from msg
        goal_quat_list = [
            self.controller_setpoint.orientation.x, 
            self.controller_setpoint.orientation.y, 
            self.controller_setpoint.orientation.z, 
            -self.controller_setpoint.orientation.w  # invert goal quat
        ]
        current_quat_list = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
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
            yaw_diff
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
        while not rospy.is_shutdown():

            if self.publish_guidance_data:

                if self.within_acceptance_margins():
                    # self.publish_guidance_data = False
                    self.goal_action_server.set_succeeded()
                    continue    # dont pub next contorller setpoint

                self.reference_model_pub.publish(self.controller_setpoint)

            self.ros_rate.sleep()

    def goal_cb(self):
        """
        Accept a goal from the guidance interface and store it as local state,
        then activate publishing
        """
        new_goal = self.goal_action_server.accept_new_goal()
        self.controller_setpoint = new_goal.target_pose.pose
        rospy.logdebug(
            "New dp guidance setpoint has type: %s" % type(self.controller_setpoint)
        )

        self.publish_guidance_data = True

    def preempt_cb(self):
        """
        The preempt callback for the action server.
        """
        if self.goal_action_server.is_preempt_requested():
            rospy.logdebug("Goal action server in dp_guidance was preempted!")
            self.goal_action_server.set_preempted()

            self.publish_guidance_data = False


if __name__ == "__main__":
    rospy.init_node("dp_guidance", log_level=rospy.DEBUG)

    try:
        dp_guidance = DPGuidance()
        dp_guidance.spin()

    except rospy.ROSInternalException as e:
        rospy.logerr(e)
