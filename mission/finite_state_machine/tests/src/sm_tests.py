#!/usr/bin/env python
import unittest
import rospy
import rostest
from nav_msgs.msg import Odometry
from geometry_msgs import PoseWithCovariance, Pose, Point
from std_msgs.msg import String
from time import sleep

class StateMachineTesting(unittest.TestCase):
    def __init__(self):
        rospy.init_node('sm_tester')
        rospy.Subscriber('state_transition', String, self.transition_cb)
        self.active_state = None
    
    def transition_cb(self,data):
        print(data.data)
        self.active_state = data.data



    def test_reach_depth(self):
        odom_pub = rospy.Publisher('odometry/filtered', Odometry, queue_size=10)
        sleep(5)
        odom_pub.publish(Odometry(None,None,PoseWithCovariance(Pose(Point(0.0,0.0,-0.5),None),None),None))
        sleep(1)
        self.assertEqual(self.active_state,'REACH_DEPTH','The state machine is not in state: "REACH_DEPTH"')




if __name__ == '__main__':
    rostest.rosrun('finite_state_machine','sm_tester',StateMachineTesting)
