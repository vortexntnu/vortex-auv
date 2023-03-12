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
    DpAction,
    DpGoal
)


def find_relative_to_mass_centre(self, offsetFromMC):
    rotationMatrix = quaternion_matrix(self.odom.pose.pose.orientation)
    return rotationMatrix.dot(offsetFromMC)

def within_acceptance_margins(self, goal):
    error = np.sqrt(pow(self.odom.pose.pose.position.x,2)+pow(self.odom.pose.pose.position.y,2))
    if(error < 0.1):
        return True
    return False

class DockingSearch(smach.State):
    def __init__(self):
        # Wait for docking_point from landmark server
        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")

    def execute(self):
        return "succeeded"
    

class DockingExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("docking").object

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # TODO: name dp_action_server
        dp_action_server = ""
        self.dp_client(actionlib.SimpleActionClient(dp_action_server, DpAction))

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self):
        self.state_pub.publish("docking/execute")

        # Power puck relative to the center of mass
        powerPuckOffset = [0,0,0.6] 

        goal = DpGoal()

        goal.x_ref = self.object.pose  

        # Shifts DP goal from Docking_point to center of mass
        goal.x_ref.position = goal.x_ref.position - find_relative_to_mass_centre(powerPuckOffset, self.odom.pose.pose.orientation) 
        goal.x_ref.position.z = self.odom.pose.pose.position.z
        
        # activates DP for all degrees of freedom
        goal.DOF = [1,1,1,1,1,1]

        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()

        while (
            not rospy.is_shutdown()
            and not self.dp_client.simple_state
            == actionlib.simple_action_client.SimpleGoalState.DONE
            and rospy.get_param("/tasks/docking")
        ):
            
            self.object = self.landmarks_client("docking").object

            
            goal.x_ref = self.object.pose   
            goal.x_ref.position = goal.x_ref.position - find_relative_to_mass_centre(powerPuckOffset, self.odom.pose.pose.orientation)

            if not within_acceptance_margins(goal):
                goal.x_ref.position.z = self.odom.pose.pose.position.z

            print(
                "DOCKING POINT DETECTED: "
                + str(goal.x_ref.Point.x)
                + ", "
                + str(goal.x_ref.Point.y)
                + ", "
                + str(goal.x_ref.Point.z)
            )
            self.dp_client.send_goal(goal)
            rate.sleep()

            if self.object.estimateFucked:
                goal.x_ref = self.odom.pose.pose
                self.dp_client.send_goal(goal)
                return "aborted"
        if (not rospy.get_param("/tasks/docking")):
            return "done"

        self.object = self.landmarks_client("docking").object
        print(
            "DOCKING POINT ESTIMATE CONVERGED AT: "
            + str(self.object.objectPose.pose.position.x)
            + "; "
            + str(self.object.objectPose.pose.position.y)
            + "; "
            + str(self.object.objectPose.pose.position.z)
        )

        starting_time = rospy.Time.now().to_sec()
        docking_duration = rospy.Duration.from_sec(15)
        while((starting_time + docking_duration) > rospy.Time.now().to_sec()):
            if (not rospy.get_param("/tasks/docking")):
                return "done"

        undocking_pose = self.odom.pose.pose
        undocking_pose.Point.z = undocking_pose.Point.z + 0.5
        undocking_pose.Quaternion = quaternion_from_euler([0, 0, 0])

        goal.x_ref = undocking_pose

        self.dp_client.send_goal(goal)

        while (
            not rospy.is_shutdown()
            and not self.dp_client.simple_state
            == actionlib.simple_action_client.SimpleGoalState.DONE
            and rospy.get_param("/tasks/docking")
        ):
            rate.sleep()

        self.dp_client.cancel_all_goals()
        
        return "done"
