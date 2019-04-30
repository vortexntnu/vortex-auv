#!/usr/bin/env python
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan
import copy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from waypoint_action_msgs.msg import WaypointAction, WaypointGoal
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PathManager(object):
    def __init__(self):
        self.global_path = Path()
        self.start = PoseStamped()
        self.goal = PoseStamped()
    def path_generator_pole_client(self):
        start = PoseStamped()
        
        tolerance = 1.0
        print "Venter paa path"
        rospy.wait_for_service('path_generator_pole')
        
        try:
            
            path_generator_pole = rospy.ServiceProxy('path_generator_pole', GetPlan)

            response = path_generator_pole(self.start,self.goal,tolerance)
            self.global_path =  response.plan
            print "path motatt"

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        

class WaypointCoordinator(object):
    def __init__(self):
        rospy.init_node('waypoint_coordinator')
        self.pub_path =  rospy.Publisher('/generated_path', Path, queue_size = 1)
        self.path_manager = PathManager()
        self.rate = rospy.Rate(10)
        

    def waypoint_action_client(self):
        waypoint_client =  actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print "Waiting"
        waypoint_client.wait_for_server()
        goal = MoveBaseGoal()
        no_of_wp = self.path_manager.global_path.poses.__len__()
        print no_of_wp

        while self.path_manager.global_path.poses.__len__() != 0:
            print self.path_manager.global_path.poses.__len__()
            #if no_of_wp == self.path_manager.global_path.poses.__len__():
            #    goal.is_first_wp = True
            #else:
            #    goal.is_first_wp = False

            goal.target_pose = self.path_manager.global_path.poses.pop(0)

            #if self.path_manager.global_path.poses.__len__() == 0:
            # goal.is_last_wp = True
            print goal 
            waypoint_client.send_goal(goal)
            print "goal sent"
            waypoint_client.wait_for_result() #rospy.Duration.from_sec(10.0)
        print "ferri"
            
    def publish(self):
        while not rospy.is_shutdown():
            print WP_coord_node.path_manager.global_path.poses
            self.pub_path.publish(self.path_manager.global_path)
            self.rate.sleep( )





if __name__ == '__main__':
    try: 
        WP_coord_node = WaypointCoordinator()
        WP_coord_node.path_manager.path_generator_pole_client()
        WP_coord_node.waypoint_action_client()
        #WP_coord_node.publish()
        while not rospy.is_shutdown():
            WP_coord_node.rate.sleep()

    except rospy.ROSInterruptException:
        pass







