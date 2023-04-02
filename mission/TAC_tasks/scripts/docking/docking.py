#!/usr/bin/python3

import rospy
import smach
import actionlib
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_matrix
from landmarks.srv import request_position

from vortex_msgs.msg import (
    dpAction, 
    dpGoal,
    ObjectPosition
)


def find_relative_to_mass_centre(self, offsetFromMC):
    rotationMatrix = quaternion_matrix(self.odom.pose.pose.orientation)
    return rotationMatrix.dot(offsetFromMC)


# Checks if we are above docking station within the accepted error radius
def within_acceptance_margins(self):
    max_error = 0.2
    error = np.sqrt(
        pow(
            self.odom.pose.pose.position.x -
            self.object.objectPose.pose.position.x, 2) + pow(
                self.odom.pose.pose.position.y -
                self.object.objectPose.pose.position.y, 2))
    if (error < max_error):
        return True
    return False


# class DockingSearch(smach.State):

#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

#         self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

#         self.landmarks_client = rospy.ServiceProxy("send_positions",
#                                                    request_position)
#         rospy.wait_for_service("send_positions")
#         self.object = self.landmarks_client("docking_point").object

#         dp_action_server = "dpAction"
#         self.dp_client = actionlib.SimpleActionClient(dp_action_server,
#                                                       dpAction))

#         rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
#         self.odom = Odometry()

#     def odom_cb(self, msg):
#         self.odom = msg

#     def execute(self, userdata):
#         self.state_pub.publish("docking/search")

#         rate = rospy.Rate(10)

#         goal = dpGoal()
#         goal.x_ref = self.odom.pose.pose
#         goal.DOF = [1, 1, 1, 1, 1, 1]
#         self.dp_client.wait_for_server()
#         self.dp_client.send_goal(goal)

#         # Wait until we find the docking point
#         while not self.object.isDetected:
#             self.object = self.landmarks_client("docking_point").object

#             if (not rospy.get_param("/tasks/docking")):
#                 self.dp_client.cancel_all_goals()
#                 return 'preempted'

#             rate.sleep()

#         return 'succeeded'


class DockingExecute(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'preempted'])
        
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("docking_point").object
 
        dp_action_server = "dpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                    dpAction)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("docking/execute")

        # # Power puck relative to the center of mass (temporary until we get static transform)
        powerPuckOffset = [0, 0, 0.6]

        goal = dpGoal()
        goal.x_ref = self.object.objectPose.pose

        # Shifts DP goal from Docking_point to center of mass
        goal.x_ref.position = goal.x_ref.position - find_relative_to_mass_centre(
            powerPuckOffset, self.odom.pose.pose.orientation)
        goal.x_ref.position.z = self.odom.pose.pose.position.z

        goal.DOF = [1, 1, 1, 1, 1, 1]

        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)
        rate = rospy.Rate(10)
        rate.sleep()

        while (not rospy.is_shutdown() and not self.dp_client.simple_state
               == actionlib.simple_action_client.SimpleGoalState.DONE
               and rospy.get_param("/tasks/docking")):

            self.object = self.landmarks_client("docking_point").object

            goal.x_ref = self.object.objectPose.pose
            goal.x_ref.position = goal.x_ref.position - find_relative_to_mass_centre(
                powerPuckOffset, self.odom.pose.pose.orientation)

            if not within_acceptance_margins():
                goal.x_ref.position.z = self.odom.pose.pose.position.z

            rospy.loginfo("DOCKING POINT DETECTED: " +
                          str(goal.x_ref.Point.x) + ", " +
                          str(goal.x_ref.Point.y) + ", " +
                          str(goal.x_ref.Point.z))

            self.dp_client.send_goal(goal)

            rate.sleep()

        if (not rospy.get_param("/tasks/docking")):
            self.dp_client.cancel_all_goals()
            return 'preempted'

        self.object = self.landmarks_client("docking_point").object

        rospy.loginfo("DOCKING POINT ESTIMATE CONVERGED AT: " +
                      str(self.object.objectPose.pose.position.x) + "; " +
                      str(self.object.objectPose.pose.position.y) + "; " +
                      str(self.object.objectPose.pose.position.z))

        starting_time = rospy.Time.now().to_sec()
        docking_duration = rospy.Duration.from_sec(15)

        while ((starting_time + docking_duration) > rospy.Time.now().to_sec()):
            if (not rospy.get_param("/tasks/docking")):
                self.dp_client.cancel_all_goals()
                return 'preempted'
            rate.sleep()

        undocking_pose = self.odom.pose.pose
        undocking_pose.Point.z = undocking_pose.Point.z + 0.5
        undocking_pose.Quaternion = quaternion_from_euler([0, 0, 0])

        goal.x_ref = undocking_pose
        self.dp_client.send_goal(goal)

        while (not rospy.is_shutdown() and not self.dp_client.simple_state
               == actionlib.simple_action_client.SimpleGoalState.DONE
               and rospy.get_param("/tasks/docking")):
            rate.sleep()

        if (not rospy.get_param("/tasks/docking")):
            self.dp_client.cancel_all_goals()
            return 'preempted'

        return 'succeeded'


class DockingStandby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        dp_action_server = "dpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                    dpAction)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("docking/standby")

        goal = dpGoal()
        goal.x_ref = self.odom.pose.pose
        goal.DOF = [1, 1, 1, 1, 1, 1]
        self.dp_client.send_goal(goal)

        rate = rospy.Rate(10)
        rate.sleep()

        while (not rospy.is_shutdown() and rospy.get_param("/tasks/docking")):
            rate.sleep()

        self.dp_client.cancel_all_goals()

        return 'succeeded'
