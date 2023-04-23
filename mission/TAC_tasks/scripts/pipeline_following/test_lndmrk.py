#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Point
import actionlib
from vortex_msgs.msg import ObjectPosition


class test():

    def __init__(self):

        rospy.init_node("test")

        self.wpPub = rospy.Publisher('/object_positions_in',
                                     ObjectPosition,
                                     queue_size=1)

    def execute(self):
        print('hellow')
        p = ObjectPosition()
        #p.pose.header[]
        p.objectID = 'pipeline'

        p.objectPose.pose.position.x = -1
        p.objectPose.pose.position.y = 0
        p.objectPose.pose.position.z = -2
        # p.objectPose.pose.orientation.x = 0
        # p.objectPose.pose.orientation.y = 0
        # p.objectPose.pose.orientation.z = 0
        # p.objectPose.pose.orientation.w = 1

        rate = rospy.Rate(10)
        t_start = rospy.get_rostime()
        t = rospy.get_rostime()
        while not rospy.is_shutdown() and t_start.secs + 10 > t.secs:
            t = rospy.get_rostime()

            if t_start.secs + 7 <= t.secs:
                p.isDetected = True
                print('isDetected = True')
            else:
                p.isDetected = False
                print('isDetected = False')
            self.wpPub.publish(p)
            rate.sleep()


if __name__ == "__main__":

    test = test()
    while not rospy.is_shutdown():
        test.execute()
