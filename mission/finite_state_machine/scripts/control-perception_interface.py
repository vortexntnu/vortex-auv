#!/usr/bin/env python

import rospy

def callback(data):
    
 
def control_perception_interface():

    pub = rospy.Publisher('object_positions', Point, queue_size=10)

    rospy.init_node('control/perception interface', anonymous=True)    

       

    while not rospy.is_shutdown():
        
        rospy.loginfo("publishing goal position")    

        pub.publish(goal_position)

        rate.sleep()
 
if __name__ == '__main__':

    try:

        goal_position_publisher()

    except rospy.ROSInterruptException:

        pass