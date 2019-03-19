#!/usr/bin/env python

#Generating path around pole
import rospy
import copy
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from waypoint_msgs.msg import Waypoint
import math
import tf
import time

class WaypointGeneratorPole(object):
    def __init__(self):
        self.start_pose = PoseStamped()
        self.pole_coordinate = PoseStamped()
        #initalize node
        rospy.init_node('setpoint_generator_pole_node')
        #Decleare server
        self.plan_server = rospy.Service('path_generator_pole', GetPlan, self.plan_server_callback)
        #Subscriber
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=1)
        #Initialize distance (Will subscribe from camera-vision computation)
        self.distance = 10
        #Spin
        self.spin = rospy.spin()
        self.quaternion = (0.0, 0.0, 0.0, 0.0)
        self.euler = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self. yaw = 0.0

    def odom_callback(self, msg):
        self.roll = 0.1
        #self.start_pose.pose = msg.pose.pose

    def dist_callback(self, msg):
        self.roll = 0.1
        self.distance = msg.distance


    def plan_server_callback(self, req):
        #ASK FOR DISTANCE TO POLE
        #self.distance = rospy.wait_for_message('Camera_centering', floatXX)

        #Initial translation and rotation
        print "For wait"
        odom_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
        print "etter"
        self.start_pose.pose = odom_msg.pose.pose
        self.quaternion = (self.start_pose.pose.orientation.x, self.start_pose.pose.orientation.y, self.start_pose.pose.orientation.z, self.start_pose.pose.orientation.w)
        self.euler = tf.transformations.euler_from_quaternion(self.quaternion)
        self.roll = self.euler[0]
        self.pitch = self.euler[1]
        self.yaw = self.euler[2]

        #Generate path
        path =  self.path_generator(req)
        return path

    def generate_coordinate_around_pole(self, i):
        new_pose = copy.deepcopy(self.pole_coordinate)
        print "HER"
        if i == 0:
            new_pose.pose.position.x += 2*math.cos(self.yaw + (math.pi/2))
            new_pose.pose.position.y += 2*math.sin(self.yaw + (math.pi/2))
            print new_pose.pose

        if i == 1:
            new_pose.pose.position.x += 2*math.cos(self.yaw)
            new_pose.pose.position.y += 2*math.sin(self.yaw)
            print new_pose.pose

        if i == 2:
            new_pose.pose.position.x += 2*math.cos(self.yaw + (3*math.pi/2))
            new_pose.pose.position.y += 2*math.sin(self.yaw + (3*math.pi/2))
            print new_pose.pose
        print new_pose.pose
        return new_pose    


    def path_generator(self, req):
        path_msg = Path()
        self.pole_coordinate.pose.position.x = self.start_pose.pose.position.x + self.distance*math.cos(self.yaw)
        self.pole_coordinate.pose.position.y = self.start_pose.pose.position.y + self.distance*math.sin(self.yaw)
        path_msg.poses.append(self.start_pose)
        

        for i in range(3):
            rospy.loginfo("HER 3.0")
            pose = self.generate_coordinate_around_pole(i)
            path_msg.poses.append(pose)

        path_msg.poses.append(self.start_pose)
        return path_msg
        
            
        
if __name__ == '__main__':
    try:
        waypoint_generator_service = WaypointGeneratorPole()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass


