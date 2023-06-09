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


class PipelineStandby(smach.State):

    def __init__(self, userdata):
        self.task = "pipeline"

        smach.State.__init__(self,
                             outcomes=["aborted", "succeeded"],
                             input_keys=['isEnabled'],
                             output_keys=['isEnabled'])

        # task manager
        self.task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=5,
            config_callback=lambda config: self.task_manager_cb(
                config, userdata))

        # landmark server
        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(self.task).object
        self.isDetected = self.object.isDetected

        # Enable Dp
        rospy.set_param("/DP/Enabled", True)
        self.dp_client = actionlib.SimpleActionClient("DpAction", dpAction)

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # Information about the current pose of Beluga
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry

    def task_manager_cb(self, config, userdata):
        rospy.loginfo(
            """Client: state change request: {Tac_states}""".format(**config))
        activated_task_id = config["Tac_states"]

        if defines.Tasks.pipeline_inspection.id == activated_task_id:
            userdata.isEnabled = True
        else:
            userdata.isEnabled = False
        print(f"isEnabled: {userdata.isEnabled} ")

        return config

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        rospy.loginfo("Standby")

        # Feedback of the current state in state machine
        self.state_pub.publish(f"{self.task}/standby")

        if userdata.isEnabled != True:
            rospy.loginfo('PIPELINE FOLLOWING ENDED')
            return "aborted"

        # hold current position
        dp_goal = dpGoal()
        dp_goal.DOF = [True, True, True, False, False, True]
        dp_goal.x_ref = self.odom.pose.pose
        self.dp_client.send_goal(dp_goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and userdata.isEnabled:
            rospy.loginfo("Standby")
            self.object = self.landmarks_client(
                self.task).object  # requesting update on the object
            self.isDetected = self.object.isDetected
            if self.isDetected:
                return "succeeded"
            rate.sleep()

        rospy.loginfo('PIPELINE FOLLOWING ENDED')
        return "aborted"
