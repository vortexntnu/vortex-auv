#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
import time
 
def goal_position_publisher():

    pub = rospy.Publisher('goal_position', Point, queue_size=10)

    rospy.init_node('perception', anonymous=True)

    rate = rospy.Rate(1) # 1hz    

    goal_position = Point(7,1.5,-0.5)        

    while not rospy.is_shutdown():
        
        rospy.loginfo("publishing goal position")    

        pub.publish(goal_position)

        rate.sleep()
 
if __name__ == '__main__':

    try:

        goal_position_publisher()

    except rospy.ROSInterruptException:

        pass