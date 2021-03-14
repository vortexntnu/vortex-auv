#!/usr/bin/env python
import rospy
import actionlib
from vortex_msgs.msg import MoveAction

class AUV_Action:
    def __init__(self):       

        self.action_server = actionlib.SimpleActionServer('/guidance/move', MoveAction, self.as_cb, auto_start=False)
        self.action_server.start()

    def as_cb(self, move_goal):
        self.action_server.set_succeeded()    


if __name__ == '__main__':

    rospy.init_node('test_action_server')
    action_server = AUV_Action()
    rospy.spin()

    

    
