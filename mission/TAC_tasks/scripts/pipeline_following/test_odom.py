#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
import actionlib


class test():

    def __init__(self):

        rospy.init_node("test")
        """ rospy.wait_for_service(
        "send_positions"
        ) """

        self.pub = rospy.Publisher("/odometry/filtered",
                                   Odometry,
                                   queue_size=1)

    def execute(self):

        goal = Odometry()
        self.pub.publish(goal)
        rate = rospy.Rate(10)
        rate.sleep()


if __name__ == "__main__":

    test = test()
    while not rospy.is_shutdown():
        test.execute()
