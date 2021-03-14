#!/usr/bin/env python
import unittest
import rospy
import rostest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from std_msgs.msg import String
from time import sleep
import subprocess
from os import setsid
from sys import stderr, stdout

        
class AUV_Mock:
    def __init__(self):        

        rospy.Subscriber('state_transition', String, self.transition_cb)        

        self.odom_pub = rospy.Publisher('odometry/filtered', Odometry, queue_size=10)

        self.current_state = None   
    

    
    def transition_cb(self,data):
        print(data.data)
        self.current_state = data.data
    
    def mock_reach_depth(self):
        self.odom_pub.publish(Odometry(None,None,PoseWithCovariance(Pose(Point(0.0,0.0,-0.5),None),None),None))
        
    
    def mock_gate_search(self):
        gp_pub = rospy.Publisher('goal_position', Point, queue_size=10)
        gp_pub.publish(Point(7,1.5,-0.5))


    

class StateMachineTesting(unittest.TestCase):

    def setup(self):
        rospy.init_node('sm_tester')
        #self.AUV_M = AUV_Mock()
        pass
        #os.system('roslaunch finite_state_machine fsm_test.launch')

    # def test_reach_depth(self):        
    #     self.AUV_Mock.mock_reach_depth()
    #     sleep(1)
    #     self.assertEqual(self.AUV_Mock.current_state,'REACH_DEPTH','The state machine is not in state: "REACH_DEPTH"')

    def test_gate_search(self):
        AUV_M = AUV_Mock()
        AUV_M.mock_reach_depth()
        #self.AUV_Mock.mock_gate_search()
        self.assertEqual(AUV_M.current_state,'GATE_SEARCH','The state machine is not in state: "GATE_SEARCH"')

        




if __name__ == '__main__': 
    rospy.init_node('sm_tester')

    # process = subprocess.Popen(['roslaunch', 'finite_state_machine', 'fsm_test.launch'],
    #                 stdout=subprocess.PIPE, 
    #                 stderr=subprocess.PIPE)
    # stdout, stderr = process.communicate()
    # stdout, stderr

    process = subprocess.Popen(['roslaunch finite_state_machine fsm_test.launch'], shell=True, preexec_fn=setsid, stdout=stdout, stderr=stderr)
    
    sleep(5)
    rostest.rosrun('finite_state_machine','sm_tests',StateMachineTesting)

    process.kill()