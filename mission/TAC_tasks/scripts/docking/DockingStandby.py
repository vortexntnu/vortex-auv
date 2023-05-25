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

class DockingStandby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.task = "docking"

        self.task_manager_client = dynamic_reconfigure.client.Client(
            "/task_manager/server",
            timeout=3,
            config_callback=self.task_manager_cb)
        self.isEnabled = True

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        dp_action_server = "DpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                      dpAction)
        self.dp_client.wait_for_server()

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def task_manager_cb(self, config):
        activated_task_id = config["Tac_states"]

        if defines.Tasks.docking.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        rospy.logwarn(f"isEnabled: {self.isEnabled} ")

        return config

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
