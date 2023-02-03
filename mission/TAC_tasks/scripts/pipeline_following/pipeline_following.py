#!/usr/bin/python3

import rospy
import smach
from landmarks.srv import request_position
from vortex_msgs.msg import (
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

        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

    # Callback function for the position subscriber
    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        
        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/converge")


class PipelineExecute(smach.State):
    def __init__(self):
        self.task = "pipeline"

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

    def execute(self, userdata):

        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/execute")