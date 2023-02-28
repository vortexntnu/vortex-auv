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
        goal.waypoints = [Point(6, 0, 0)]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(10)
        rate.sleep()

    def stop(self):
        self.vtf_client.cancel_all_goals()

if __name__ == "__main__":
    #print('hello')
    test = test()
    while True:
        print('hello')
        test.stop()
        