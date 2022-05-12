#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry

rospy.init_node("fake_odom", log_level=rospy.DEBUG)
ros_rate = rospy.Rate(100)

pub = rospy.Publisher("/odometry/filtered", Odometry)

odom = Odometry()
odom.pose.pose.position.x = 0.0
odom.pose.pose.position.y = 0.0
odom.pose.pose.position.z = 0.0
odom.pose.pose.orientation.x = 0.0
odom.pose.pose.orientation.y = 1.0
odom.pose.pose.orientation.z = 0.0
odom.pose.pose.orientation.w = 0.0

odom.twist.twist.linear.x = 0.0
odom.twist.twist.linear.y = 0.0
odom.twist.twist.linear.z = 0.0
odom.twist.twist.angular.x = 0.0
odom.twist.twist.angular.y = 0.0
odom.twist.twist.angular.z = 0.0


while not rospy.is_shutdown():
    pub.publish(odom)

    ros_rate.sleep()
