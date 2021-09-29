#!/usr/bin/env python


import rospy

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

def length_of_bounding_box(bbox):
    return xmax - xmin

def hight_of_bounding_box(bbox):
    return ymax - ymin


class SizeEstimatorNode():
    def __init__(self):
        rospy.init_node('size_estimator')
        self.estimatorSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimator', BoundingBox, queue_size= 1)


    def callback(self, data):
        self.min_x = data.bounding_boxes[0].xmin + 123
        self.var = data
        self.var.xmin = self.min_x
        estimatorPub.publish(self.var)
    

if __name__ == '__main__':
    node = SizeEstimatorNode()
    
    while not rospy.is_shutdown():
        rospy.spin()



