#!/usr/bin/env python
from move_base_msgs.msg import MoveBaseAction
import actionlib
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
import time


class WaypointActionServer(object):
    def __init__(self):
        self.WP_server = actionlib.SimpleActionServer('move_base', MoveBaseAction, self.recieve_WP, False)
        self.WP_server.start()
        print "HEI"

    def recieve_WP(self,goal):
        print goal
        time.sleep(3)
        self.WP_server.set_succeeded()



if __name__ == '__main__':
    try: 
        rospy.init_node('fake_guidance')
        serv = WaypointActionServer()
        rospy.spin()        

    except rospy.ROSInterruptException:
        pass