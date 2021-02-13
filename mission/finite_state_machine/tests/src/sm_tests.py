#!/usr/bin/env python
import unittest
import rospy
import rostest
from nav_msgs.msg import Odometry
from geometry_msgs import PoseWithCovariance, Pose, Point
from time import sleep

class StateMachineTesting(unittest.TestCase):

    def test_reach_depth(self):
        rospy.init_node('sm_tester')
        odom_pub = rospy.Publisher('odometry/filtered', Odometry, queue_size=10)
        sleep(5)
        odom_pub.Publish(Odometry(None,None,PoseWithCovariance(Pose(Point(0.0,0.0,-0.5),None),None),None))
        sleep(1)
        #self.assert




if __name__ == '__main__':
