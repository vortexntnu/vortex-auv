#!/usr/bin/python3

import time
import rospy
import smach
import actionlib
from landmarks.srv import request_position
from vortex_msgs.msg import (
    VtfPathFollowingAction,
    VtfPathFollowingGoal,
    ObjectPosition,
    DpSetpoint
)
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class PipelineConverge(smach.State):
    def __init__(self):
        self.task = "pipeline"

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(f"{self.task}").object

        # TODO meldingstype for DP er ikke bestemt enda
        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)
        
        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # Information about the current pose of Beluga
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

        smach.State.__init__(self, outcomes=["succeeded"])

    # Callback function for the position subscriber
    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):

        rospy.loginfo("Entering PipelineConverge")
        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/converge")

        # Converge to correct height above pipeline and heading
        #dp_goal = DpSetpoint()
        #dp_goal.control_mode = 7  # POSE_HOLD
        #dp_goal.setpoint = self.object.objectPose
        #self.dp_pub.publish(dp_goal)

        time.sleep(3)
        return "succeeded"

class PipelineExecute(smach.State):
    def __init__(self):
        self.task = "pipeline"

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(f"{self.task}").object
        
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

        smach.State.__init__(self, outcomes=["aborted"])

    def execute(self, userdata):
        rospy.loginfo("Entering PipelineExecute")

        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/execute")

        # constant vtf parameters
        goal = VtfPathFollowingGoal()
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"

        rate = rospy.Rate(10)
        while self.object.isDetected and rospy.get_param("/tasks/pipeline_inspection"):
            print(
                "PATH POSITION DETECTED: "
                + str(self.object.objectPose.pose.position.x)
                + ", "
                + str(self.object.objectPose.pose.position.y)
                + ", "
                + str(self.object.objectPose.pose.position.z)
                + " isDetected: "
                + str(self.object.isDetected)
            )

            # TODO: should have an if statement that maintain altitude above pipeline??

            goal.waypoints = [self.object.objectPose.pose.position]
            self.vtf_client.wait_for_server()
            rospy.loginfo("Connection with vtf server")
            self.vtf_client.send_goal(goal)
            rate.sleep()
        
        return "aborted"

class PipelineStandby(smach.State):
    def __init__(self):
        self.task = "pipeline"

        # TODO meldingstype for DP er ikke bestemt enda
        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)
        
        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # Information about the current pose of Beluga
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

        #smach.State.__init__(self)
        smach.State.__init__(self, outcomes=["aborted"])

    # Callback function for the position subscriber
    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):

        rospy.loginfo("Standby")
        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/standby")

        # hold current position
        # dp_goal = DpSetpoint()
        # dp_goal.control_mode = 7  # POSE_HOLD
        # dp_goal.setpoint = self.odom.pose.pose
        # self.dp_pub.publish(dp_goal)

        rate = rospy.Rate(10)
        while(): #rospy.get_param("/tasks/pipeline_inspection")
            rospy.loginfo("Standby")
            rate.sleep()
        
        self.request_preempt()
        return "aborted"