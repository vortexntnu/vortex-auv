#!/usr/bin/python3

import rospy
import actionlib
from geometry_msgs.msg import Point
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal
from nav_msgs.msg import Odometry


class FourPoint():
    def __init__(self):
        rospy.init_node("four_point_node")

        self.odom = Odometry()
        self.goal = VtfPathFollowingGoal()

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

    def odom_cb(self, msg):
        self.odom = msg

    def spin(self):
        z = self.odom.pose.pose.position.z - 0.3
        self.goal.waypoints = [Point(1.5, 0, z), Point(1.5, 1.5, z), Point(0, 1.5, z), Point(0, 0, z)]
        self.goal.forward_speed = rospy.get_param("fsm/medium_speed")
        self.goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(self.goal)





if __name__ == "__main__":
    while not rospy.is_shutdown:
        four_point = FourPoint()
        four_point.spin()



