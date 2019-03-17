#!/usr/bin/env python

#Generating path around pole
import rospy
import copy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from waypoint_msgs.msg import Waypoint
import math
import tf
import time

class WaypointGeneratorPole(object):
    def __init__(self):
        #initalize node
        rospy.init_node('setpoint_generator_pole_node')

        self.rate = rospy.Rate(10)
        #Subscriber
        self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback, queue_size=1)

        #Publisher
        self.pub = rospy.Publisher('/generated_path', Path, queue_size = 1)

        #Initialize distance (Will subscribe from camera-vision computation)
        self.distance = 10

        #Initalize flag
        self.odom_flag = 0
        self.path_flag  = 0
        
        #initialize pose
        self.start_pose = PoseStamped()
        self.quaternion = (0.0, 0.0, 0.0, 0.0)
        self.euler = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self. yaw = 0.0

        # Init pole coordinate
        self.pole_coordinate = PoseStamped()

        #init path
        self.path_msg = Path()
        
    def odom_callback(self, msg):
        self.start_pose.pose = msg.pose.pose

        self.quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self.euler = tf.transformations.euler_from_quaternion(self.quaternion)
        self.roll = self.euler[0]
        self.pitch = self.euler[1]
        self.yaw = self.euler[2]

        self.path_generator()

        self.odom_flag = 1

    def dist_callback(self, msg):
        self.distance = msg.distance

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

    def path_generator(self):
        rospy.loginfo("HER")
        self.pole_coordinate.pose.position.x = self.start_pose.pose.position.x + self.distance*math.cos(self.yaw)
        self.pole_coordinate.pose.position.y = self.start_pose.pose.position.y + self.distance*math.sin(self.yaw)
        rospy.loginfo("HER 2.0")
        self.path_msg.poses.append(self.start_pose)
        

        for i in range(3):
            rospy.loginfo("HER 3.0")
            pose = self.generate_coordinate_around_pole(i)
            self.path_msg.poses.append(pose)

        self.path_msg.poses.append(self.start_pose)
        rospy.loginfo("HER 4.0")
        self.pub.publish(self.path_msg)

        print self.path_msg.poses.__len__()
        print self.path_msg.poses[0].pose.position.x
        print self.path_msg.poses[1].pose.position.x
        print self.path_msg.poses[2].pose.position.x
        print self.path_msg.poses[3].pose.position.x
        print self.path_msg.poses[4].pose.position.x

        rospy.loginfo("HER 5.0")
        rospy.signal_shutdown('Path generated')
    
            
        
if __name__ == '__main__':
    try:
        waypoint_generator_pole_node = WaypointGeneratorPole()
        while not rospy.is_shutdown():
            waypoint_generator_pole_node.rate.sleep()

    except rospy.ROSInterruptException:
        pass


