#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Point
import actionlib
from vortex_msgs.msg import (
    VtfPathFollowingAction,
    VtfPathFollowingGoal,
    DpSetpoint,
    SetVelocityGoal,
    SetVelocityAction,
)

class test():
    def __init__(self):

        rospy.init_node("test")

        """ rospy.wait_for_service(
        "send_positions"
        ) """

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

    def execute(self):

        goal = VtfPathFollowingGoal()
        goal.waypoints = [Point(0, 10, -1)]
        goal.forward_speed = 0.2 #rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"
        print('1')
        self.vtf_client.wait_for_server()
        print('2')
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(10)
        rate.sleep()

    def stop(self):
        self.vtf_client.cancel_all_goals()

if __name__ == "__main__":

    test = test()
    while not rospy.is_shutdown():
        test.execute()
        