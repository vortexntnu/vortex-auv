#!/usr/bin/python3

import rospy
import smach
import actionlib
import numpy as np
import math
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose, Point, Wrench, Quaternion, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
from landmarks.srv import request_position
from task_manager_defines import defines
from vortex_msgs.msg import (dpAction, dpGoal, dpResult, ObjectPosition)

from task_manager_defines import defines
from task_manager_client.TaskManagerClient import TaskManagerClient  # type: ignore

from dp_client_py.DPClient import DPClient


class DockingStandby(smach.State):

    def __init__(self, dp_client_handle):
        smach.State.__init__(self, outcomes=['succeeded'])

        # Task Manager client for docking
        self.task_manager_client = TaskManagerClient(defines.Tasks.docking.id)
        self.task_manager_client.is_enabled = False

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # DP action client
        self.dp_client = dp_client_handle
        self.current_goal_pose = Pose()

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("docking/standby")

        goal = dpGoal()
        goal.x_ref = self.odom.pose.pose
        goal.DOF = [True, True, True, False, False, True]
        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)

        rate = rospy.Rate(10)
        rate.sleep()

        rospy.set_param("/DP/Enable", True)

        # Stays in standby mode until Task manager deactivates the docking task
        while not rospy.is_shutdown() and self.isEnabled:
            rate.sleep()

        rospy.set_param("/DP/Enable", False)

        return 'succeeded'
