#!/usr/bin/python3

import rospy
import smach
from landmarks.srv import request_position
from vortex_msgs.msg import DpSetpoint
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class PipelineExecute(smach.State):

    def __init__(self):
        self.task = "pipeline"

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(f"{self.task}").object

        self.dp_pub = rospy.Publisher("/controllers/dp_data",
                                      DpSetpoint,
                                      queue_size=1)

        # Information about the current pose of Beluga
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

        smach.State.__init__(self, outcomes=["aborted"])

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        rospy.loginfo("Entering PipelineExecute")

        self.object = self.landmarks_client(f"{self.task}").object

        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/execute")

        # constant vtf parameters
        goal = DpSetpoint()
        goal.control_mode = 5  # POSITION_HEADING_HOLD

        rate = rospy.Rate(10)
        while not rospy.is_shutdown(
        ) and self.object.isDetected:  # and rospy.get_param("/tasks/pipeline_inspection"):
            print("PATH POSITION DETECTED: " +
                  str(self.object.objectPose.pose.position.x) + ", " +
                  str(self.object.objectPose.pose.position.y) + ", " +
                  str(self.object.objectPose.pose.position.z) +
                  " isDetected: " + str(self.object.isDetected))

            # TODO: should have an if statement that maintain altitude above pipeline??

            goal.setpoint = [self.object.objectPose.pose]
            goal.setpoint.position.z = self.odom.pose.pose.position.z
            self.dp_pub.publish(goal)
            self.object = self.landmarks_client(
                f"{self.task}").object  # requesting new points
            rate.sleep()

        return "aborted"


class PipelineStandby(smach.State):

    def __init__(self):
        self.task = "pipeline"

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(f"{self.task}").object

        self.dp_pub = rospy.Publisher("/controllers/dp_data",
                                      DpSetpoint,
                                      queue_size=1)

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # Information about the current pose of Beluga
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

        #smach.State.__init__(self)
        smach.State.__init__(self, outcomes=["aborted", "succeeded"])

    # Callback function for the position subscriber
    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):

        rospy.loginfo("Standby")
        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/standby")

        # hold current position
        dp_goal = DpSetpoint()
        dp_goal.control_mode = 7  # POSE HOLD
        dp_goal.setpoint = self.odom.pose.pose
        self.dp_pub.publish(dp_goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown(
        ):  # and rospy.get_param("/tasks/pipeline_inspection"):
            rospy.loginfo("Standby")
            self.object = self.landmarks_client(
                f"{self.task}").object  # requesting update on the object
            if self.object.isDetected:
                return "succeeded"
            rate.sleep()

        return "aborted"
