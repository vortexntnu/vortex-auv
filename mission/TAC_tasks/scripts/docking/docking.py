#!/usr/bin/python3

import rospy
import smach
import actionlib
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, quaternion_matrix
from landmarks.srv import request_position

from vortex_msgs.msg import (dpAction, dpGoal, dpResult, ObjectPosition)

# class DockingSearch(smach.State):

#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

#         self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

#         self.landmarks_client = rospy.ServiceProxy("send_positions",
#                                                    request_position)
#         rospy.wait_for_service("send_positions")
#         self.object = self.landmarks_client("docking_point").object

#         dp_action_server = "DpAction"
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
#         self.dp_client.send_goal(goal)

#         # Wait until we find the docking point
#         while not self.object.isDetected:
#             self.object = self.landmarks_client("docking_point").object

#             if (not rospy.get_param("/tasks/docking")):
#                 rospy.set_param("/DP/Enabled", False)
#                 return 'preempted'

#             rate.sleep()

#         return 'succeeded'


def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

    return rot_matrix


class DockingExecute(smach.State):

    def __init__(self):

        # Enable Dp
        rospy.set_param("/DP/Enabled", True)

        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("docking_point").object

        dp_action_server = "DpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                      dpAction)

        rospy.loginfo("after dp")

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

        rospy.loginfo("after odom")

    def odom_cb(self, msg):
        self.odom = msg

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

    def find_relative_to_mass_centre(self, offsetFromMC):
        tf_quaternion = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w
        ]

        rotationMatrix = quaternion_rotation_matrix(tf_quaternion)
        offsetArray = rotationMatrix.dot(offsetFromMC)

        offsetPoint = Point()
        offsetPoint.x = offsetArray[0]
        offsetPoint.y = offsetArray[1]
        offsetPoint.z = offsetArray[2]

        return offsetPoint

    def execute(self, userdata):

        rate = rospy.Rate(2)

        # rospy.loginfo("10 seconds to start")

        # rospy.sleep(10.)

        # rospy.loginfo("Starting")

        self.state_pub.publish("docking/execute")

        # # Power puck relative to the center of mass (temporary until we get static transform)
        powerPuckOffset = [0, 0, -0.6]

        goal = dpGoal()
        goal.x_ref = self.object.objectPose.pose

        # Shifts DP goal from Docking_point to center of mass
        offsetPoint = self.find_relative_to_mass_centre(powerPuckOffset)
        goal.x_ref.position.x = goal.x_ref.position.x - offsetPoint.x
        goal.x_ref.position.y = goal.x_ref.position.y - offsetPoint.y
        goal.x_ref.position.z = self.odom.pose.pose.position.z

        goal.DOF = [True, True, True, True, True, True]

        self.dp_client.send_goal(goal)
        rate.sleep()

        while (not rospy.is_shutdown()  #and not self.dp_client.simple_state
               #        == actionlib.simple_action_client.SimpleGoalState.DONE
               and rospy.get_param("/tasks/docking")):

            self.object = self.landmarks_client("docking_point").object

            goal.x_ref = self.object.objectPose.pose

            offsetPoint = self.find_relative_to_mass_centre(powerPuckOffset)
            goal.x_ref.position.x = goal.x_ref.position.x - offsetPoint.x
            goal.x_ref.position.y = goal.x_ref.position.y - offsetPoint.y
            goal.x_ref.position.z = goal.x_ref.position.z - offsetPoint.z

            if not self.within_acceptance_margins():
                goal.x_ref.position.z = self.odom.pose.pose.position.z

            rospy.loginfo("DOCKING POINT DETECTED: " +
                          str(goal.x_ref.position.x) + ", " +
                          str(goal.x_ref.position.y) + ", " +
                          str(goal.x_ref.position.z))

            self.dp_client.send_goal(goal)

            rate.sleep()

        if (not rospy.get_param("/tasks/docking")):
            rospy.set_param("/DP/Enabled", False)
            return 'preempted'

        self.object = self.landmarks_client("docking_point").object

        rospy.loginfo("DOCKING POINT ESTIMATE CONVERGED AT: " +
                      str(self.object.objectPose.pose.position.x) + "; " +
                      str(self.object.objectPose.pose.position.y) + "; " +
                      str(self.object.objectPose.pose.position.z))

        # starting_time = rospy.Time.now().to_sec()
        # docking_duration = rospy.Duration.from_sec(15)

        rospy.loginfo("Docked to station")

        # while ((starting_time + docking_duration) > rospy.Time.now().to_sec()):
        #     rospy.loginfo("Waiting")
        #     if (not rospy.get_param("/tasks/docking")):
        #         rospy.set_param("/DP/Enabled", False)
        #         return 'preempted'
        #     rate.sleep()

        rospy.loginfo("Leaving docking station")

        undocking_pose = self.odom.pose.pose
        undocking_pose.position.z = undocking_pose.position.z + 0.5
        # undocking_pose.orientation = quaternion_from_euler(0, 0, 0)

        goal.x_ref = undocking_pose
        self.dp_client.send_goal(goal)

        rospy.loginfo("test")

        while (not rospy.is_shutdown() and not self.dp_client.simple_state
               == actionlib.simple_action_client.SimpleGoalState.DONE
               and rospy.get_param("/tasks/docking")):
            rate.sleep()

        if (not rospy.get_param("/tasks/docking")):
            rospy.set_param("/DP/Enabled", False)
            return 'preempted'

        return 'succeeded'


class DockingStandby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        dp_action_server = "DpAction"
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
        goal.DOF = [True, True, True, True, True, True]
        self.dp_client.send_goal(goal)

        rate = rospy.Rate(10)
        rate.sleep()

        while (not rospy.is_shutdown() and rospy.get_param("/tasks/docking")):
            rate.sleep()

        rospy.set_param("/DP/Enabled", False)

        return 'succeeded'
