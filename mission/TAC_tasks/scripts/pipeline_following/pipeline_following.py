#!/usr/bin/python3

import rospy
import smach
from landmarks.srv import request_position
from vortex_msgs.msg import dpAction, dpGoal, dpResult
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import actionlib
from task_manager_defines import defines
import dynamic_reconfigure.client


class PipelineExecute(smach.State):

    def __init__(self):
        self.task = "pipeline"

        # task manager
        self.isEnabled = False
        task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=5,
            config_callback=self.task_manager_cb)

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # landmark server
        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(f"{self.task}").object
        self.isDetected = self.object.isDetected

        # Enable Dp
        rospy.set_param("/DP/Enabled", True)
        self.dp_client = actionlib.SimpleActionClient("DpAction", dpAction)
        rospy.Subscriber("/DpAction/result", dpResult, self.dp_goal_cb)
        self.reached_goal = False

        # Information about the current pose of Beluga
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

        smach.State.__init__(self, outcomes=["aborted"])

    def task_manager_cb(self, config):
        rospy.loginfo(
            """Client: state change request: {Tac_states}""".format(**config))
        activated_task_id = config["Tac_states"]

        if defines.Tasks.valve_vertical.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"isEnabled: {self.isEnabled} ")

        return config

    def odom_cb(self, msg):
        self.odom = msg

    def dp_goal_cb(self, msg):
        self.reached_goal = msg.result.finished

    def execute(self, userdata):
        rospy.loginfo("Entering PipelineExecute")
        # Feedback of the current state
        self.state_pub.publish(f"{self.task}/execute")

        if self.isEnabled == False:
            return "aborted"

        # object request
        self.object = self.landmarks_client(f"{self.task}").object
        self.isDetected = self.object.isDetected

        # DP goal setup
        goal = dpGoal()
        goal.DOF = [True, True, True, False, False, True]
        goal.x_ref = self.object.objectPose.pose
        goal.x_ref.position.z = self.odom.pose.pose.position.z
        self.dp_client.send_goal(goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.isDetected and self.isEnabled:
            print("PATH POSITION DETECTED: " +
                  str(self.object.objectPose.pose.position.x) + ", " +
                  str(self.object.objectPose.pose.position.y) + ", " +
                  str(self.object.objectPose.pose.position.z) +
                  " isDetected: " + str(self.isDetected))

            # TODO: should have an if statement that maintain altitude above pipeline??

            # send new goal only if previous goal is reached
            if self.reached_goal:
                self.dp_client.send_goal(goal)

            # Update DP goal
            self.object = self.landmarks_client(
                f"{self.task}").object  # requesting new point
            self.isDetected = self.object.isDetected
            goal.x_ref = self.object.objectPose.pose
            goal.x_ref.position.z = self.odom.pose.pose.position.z

            rate.sleep()

        return "aborted"


class PipelineStandby(smach.State):

    def __init__(self):
        self.task = "pipeline"

        # task manager
        task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=5,
            config_callback=self.task_manager_cb)

        # landmark server
        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(f"{self.task}").object
        self.isDetected = self.object.isDetected

        # Enable Dp
        rospy.set_param("/DP/Enabled", True)
        self.dp_client = actionlib.SimpleActionClient("DpAction", dpAction)

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # Information about the current pose of Beluga
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

        smach.State.__init__(self, outcomes=["aborted", "succeeded"])

    def task_manager_cb(self, config):
        rospy.loginfo(
            """Client: state change request: {Tac_states}""".format(**config))
        activated_task_id = config["Tac_states"]

        if defines.Tasks.valve_vertical.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"isEnabled: {self.isEnabled} ")

        return config

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        rospy.loginfo("Standby")
        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/standby")

        if self.isEnabled == False:
            return "aborted"

        # hold current position
        dp_goal = dpGoal()
        dp_goal.DOF = [True, True, True, False, False, True]
        dp_goal.x_ref = self.odom.pose.pose
        self.dp_client.send_goal(dp_goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.isEnabled:
            rospy.loginfo("Standby")
            self.object = self.landmarks_client(
                f"{self.task}").object  # requesting update on the object
            self.isDetected = self.object.isDetected
            if self.isDetected:
                return "succeeded"
            rate.sleep()

        return "aborted"
