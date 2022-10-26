import rospy
import smach
from geometry_msgs.msg import Point

import actionlib
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal
from nav_msgs.msg import Odometry


class ReachDepth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        goal = VtfPathFollowingGoal()
        position = Point()
        position.x = self.odom.pose.pose.position.x + 0.1
        position.y = self.odom.pose.pose.position.y
        position.z = -1.0
        goal.waypoints = [position]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (
                self.vtf_client.simple_state
                == actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                print("ReachDepth succeeded")
                break
            rate.sleep()

        return "succeeded"
