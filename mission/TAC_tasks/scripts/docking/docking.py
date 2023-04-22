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
from tf.transformations import quaternion_from_euler, quaternion_matrix
from landmarks.srv import request_position
from task_manager_defines import defines
from vortex_msgs.msg import (dpAction, dpGoal, dpResult, ObjectPosition)


def quaternion_rotation_matrix(Q):

    # Convert a quaternion into a full three-dimensional rotation matrix.

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


def distance_between_points(p1, p2):
    return math.dist([p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z])


class DockingExecute(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

        self.task = "docking"
        self.task_manager_client = dynamic_reconfigure.client.Client("/task_manager/server", timeout=3, config_callback=self.task_manager_cb)
        self.isEnabled = rospy.get_param("/tasks/docking")

        # Enable Dp
        rospy.set_param("/DP/Enable", True)

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("docking_point").object

        dp_action_server = "DpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                      dpAction)
        self.dp_client.wait_for_server()

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

        rospy.Subscriber("/DpAction/result", Odometry, self.dp_goal_cb)
        self.odom = Odometry()

        self.reached_dp_goal = False
        self.current_goal_pose = Pose()

        self.thrust_pub = rospy.Publisher(rospy.get_param("/thrust/thrust_topic"), Wrench, queue_size=1)

        # Height to converge above the docking station
        self.convergence_height = 2

    def task_manager_cb(self, config):
        activated_task_id = config["Tac_states"]

        if defines.Tasks.valve_vertical.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"Docking Enabled: {self.isEnabled} ")

        return config

    def odom_cb(self, msg):
        self.odom = msg

    def dp_goal_cb(self, msg):
        if (msg.result.finished):
            self.reached_dp_goal = True

    # Checks if we are above docking station within the accepted error radius
    def above_docking_point(self):
        max_error = 0.5
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
            self.object.objectPose.pose.orientation.x,
            self.object.objectPose.pose.orientation.y,
            self.object.objectPose.pose.orientation.z,
            self.object.objectPose.pose.orientation.w
        ]

        rotationMatrix = quaternion_rotation_matrix(tf_quaternion)
        offsetArray = rotationMatrix.dot(offsetFromMC)

        offsetPoint = Point()
        offsetPoint.x = offsetArray[0]
        offsetPoint.y = offsetArray[1]
        offsetPoint.z = offsetArray[2]

        return offsetPoint

    def should_send_new_goal(self):
        error = distance_between_points(self.object.objectPose.pose.position,
                                        self.current_goal_pose.position)
        a = 0.04
        b = 0.02
        error_limit = a * distance_between_points(
            self.odom.pose.pose.position,
            self.object.objectPose.pose.position) + b
        if error > error_limit:
            return True
        return False

    def execute(self, userdata):

        self.state_pub.publish("docking/execute")

        # # Power puck relative to the center of mass (temporary until we get static transform)
        powerPuckOffset = [0, 0, -0.6]

        goal = dpGoal()
        goal.x_ref = self.object.objectPose.pose

        # Shifts DP goal from Docking_point to center of mass
        offsetPoint = self.find_relative_to_mass_centre(powerPuckOffset)
        goal.x_ref.position.x = goal.x_ref.position.x + offsetPoint.x
        goal.x_ref.position.y = goal.x_ref.position.y + offsetPoint.y
        goal.x_ref.position.z = self.odom.pose.pose.position.z

        goal.DOF = [True, True, True, False, False, True]

        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)
        self.current_goal_pose = goal.x_ref

        rate = rospy.Rate(10)
        sending_rate = rospy.Rate(1)
        sending_rate.sleep()

        while (not rospy.is_shutdown() and self.isEnabled and not self.reached_dp_goal):

            self.object = self.landmarks_client("docking_point").object

            goal.x_ref = self.object.objectPose.pose

            offsetPoint = self.find_relative_to_mass_centre(powerPuckOffset)
            goal.x_ref.position.x = goal.x_ref.position.x + offsetPoint.x
            goal.x_ref.position.y = goal.x_ref.position.y + offsetPoint.y
            goal.x_ref.position.z = goal.x_ref.position.z + offsetPoint.z

            if not self.above_docking_point():
                goal.x_ref.position.z = goal.x_ref.position.z + self.convergence_height
                rospy.loginfo("Converging above docking point")

            rospy.loginfo("DOCKING POINT DETECTED: " +
                          str(self.object.objectPose.pose.position.x) + ", " +
                          str(self.object.objectPose.pose.position.y) + ", " +
                          str(self.object.objectPose.pose.position.z))
            if self.should_send_new_goal():
                self.dp_client.wait_for_server()
                self.dp_client.send_goal(goal)
                rospy.loginfo("Sending DP goal")
                self.current_goal_pose = goal.x_ref

            sending_rate.sleep()

        self.reached_dp_goal = False

        if not self.isEnabled:
            return 'preempted'

        self.object = self.landmarks_client("docking_point").object

        rospy.loginfo("DOCKING POINT ESTIMATE CONVERGED AT: " +
                      str(self.object.objectPose.pose.position.x) + "; " +
                      str(self.object.objectPose.pose.position.y) + "; " +
                      str(self.object.objectPose.pose.position.z))

        rospy.loginfo("BELUGA AT: " +
                      str(self.odom.pose.pose.position.x) + "; " +
                      str(self.odom.pose.pose.position.y) + "; " +
                      str(self.odom.pose.pose.position.z))
        
        rospy.set_param("/DP/Enable", False)

        downward_trust = 60         # Max limit for joystick heave
        thrust_vector = Wrench()
        thrust_vector.force.z = - downward_trust
        self.thrust_pub.publish(thrust_vector)
        
        docking_duration = 25.0
        finished_docking_time = rospy.Time.now().to_sec() + docking_duration

        rospy.loginfo("Docked to station")

        i = 0
        while (finished_docking_time > rospy.Time.now().to_sec()):
            i = i + 1
            rospy.loginfo("Waiting " + str(i))
            rate.sleep()
            if not self.isEnabled:
                thrust_vector = Wrench()
                self.thrust_pub.publish(thrust_vector)
                return 'preempted'

        rospy.loginfo("Leaving docking station")

        thrust_vector = Wrench()
        self.thrust_pub.publish(thrust_vector)

        rospy.set_param("/DP/Enable", True)

        undocking_pose = self.odom.pose.pose
        undocking_pose.position.z = undocking_pose.position.z + 0.5

        goal.x_ref = undocking_pose
        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)

        while (not rospy.is_shutdown() and self.isEnabled and not self.reached_dp_goal):
            rate.sleep()

        if not self.isEnabled:
            return 'preempted'

        return 'succeeded'


class DockingStandby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        self.task = "docking"
        
        self.task_manager_client = dynamic_reconfigure.client.Client("/task_manager/server", timeout=3, config_callback=self.task_manager_cb)
        self.isEnabled = rospy.get_param("/tasks/docking")

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        dp_action_server = "DpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                      dpAction)
        self.dp_client.wait_for_server()

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

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
        self.state_pub.publish("docking/standby")

        goal = dpGoal()
        goal.x_ref = self.odom.pose.pose
        goal.DOF = [True, True, True, False, False, True]
        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)

        rate = rospy.Rate(10)
        rate.sleep()

        while not rospy.is_shutdown() and self.isEnabled:
            rate.sleep()

        rospy.set_param("/DP/Enable", False)

        return 'succeeded'
