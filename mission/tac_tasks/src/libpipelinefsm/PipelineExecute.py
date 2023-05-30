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

    def __init__(self, userdata):
        self.task = "pipeline"

        smach.State.__init__(self,
                             outcomes=["aborted"],
                             input_keys=['isEnabled'],
                             output_keys=['isEnabled'])

        # task manager
        self.task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=5,
            config_callback=lambda config: self.task_manager_cb(
                config, userdata))

        # state information
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # landmark server
        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client(self.task).object
        self.isDetected = self.object.isDetected

        # Enable Dp
        rospy.set_param("/DP/Enabled", True)
        self.dp_client = actionlib.SimpleActionClient("DpAction", dpAction)
        rospy.Subscriber("/DpAction/result", dpResult, self.dp_goal_cb)
        self.reached_goal = False

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

    def dp_goal_cb(self, msg):
        self.reached_goal = msg.result.finished

    def execute(self, userdata):
        rospy.loginfo("Entering PipelineExecute")
        # Feedback of the current state
        self.state_pub.publish(f"{self.task}/execute")

        if userdata.isEnabled == False:
            return "aborted"

        # object request
        self.object = self.landmarks_client(self.task).object
        self.isDetected = self.object.isDetected

        # DP goal setup
        goal = dpGoal()
        goal.DOF = [True, True, True, False, False, True]
        goal.x_ref = self.object.objectPose.pose
        z_position = self.odom.pose.pose.position.z
        goal.x_ref.position.z = z_position
        self.dp_client.send_goal(goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown(
        ) and self.isDetected and userdata.isEnabled:
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
                self.task).object  # requesting new point
            self.isDetected = self.object.isDetected
            goal.x_ref = self.object.objectPose.pose
            goal.x_ref.position.z = z_position

            rate.sleep()

        return "aborted"
