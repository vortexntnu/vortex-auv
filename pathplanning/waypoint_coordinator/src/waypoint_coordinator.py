#!/usr/bin/env python
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan
import copy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from waypoint_msgs.msg import Waypoint
import rospy

class PathManager(object):
    def __init__(self):
        self.global_path = Path()
        self.start = PoseStamped()
        self.goal = PoseStamped()
        self.waypoint_counter = 0
        self.is_last_waypoint = False



class WaypointCoordinator(object):
    def __init__(self):
        rospy.init_node('waypoint_coordinator')
        self.pub_path =  rospy.Publisher('/generated_path', Path, queue_size = 1)
        self.path_manager = PathManager()
        self.rate = rospy.Rate(10)
        
        
    def path_generator_pole_client(self):
        start = PoseStamped()
        
        tolerance = 1.0
        print "Venter her"
        rospy.wait_for_service('path_generator_pole')
        
        try:
            
            path_generator_pole = rospy.ServiceProxy('path_generator_pole', GetPlan)

            response = path_generator_pole(self.path_manager.start,self.path_manager.goal,tolerance)
            self.path_manager.global_path =  response.plan
            print "motatt"

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def publish(self):
        while not rospy.is_shutdown():
            print WP_coord_node.path_manager.global_path.poses
            self.pub_path.publish(self.path_manager.global_path)
            self.rate.sleep()





if __name__ == '__main__':
    try: 
        WP_coord_node = WaypointCoordinator()
        WP_coord_node.path_generator_pole_client()
        WP_coord_node.publish()
        while not rospy.is_shutdown():
            WP_coord_node.rate.sleep()

    except rospy.ROSInterruptException:
        pass







