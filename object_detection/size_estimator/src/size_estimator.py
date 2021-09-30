#!/usr/bin/env python


import rospy

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from std_msgs.msg import Int64, Float64


class SizeEstimatorNode():
    ppdh = 29.87
    def __init__(self):
        rospy.init_node('size_estimator')
        self.estimatorSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimator', Float64, queue_size= 1)


    def callback(self, data):
        # Allocates msg data to local variables
        # In order to process abs size
        self.msgdata = data.bounding_boxes[0]
        self.min_x_pxl = self.msgdata.xmin
        self.max_x_pxl = self.msgdata.xmax

        self.min_y_pxl = self.msgdata.ymin
        self.max_y_pxl = self.msgdata.ymax

        self.depth_mtr = self.msgdata.z

        var_abs_width_obj = self.abs_width_obj([self.min_x_pxl,self.max_x_pxl,self.depth_mtr])
        var_abs_height_obj= self.abs_height_obj([self.min_y_pxl,self.max_y_pxl,self.depth_mtr])

        self.mymsg = var_abs_width_obj
        send = Float64()
        send.data = self.mymsg
        self.estimatorPub.publish(send)
    
    def abs_height_obj(self, data):
        # Do stuff
        var_abs_height_obj = 0
        return var_abs_height_obj

    def abs_width_obj(self, data):
        # Do stuff
        F = 0.0028
        p = data[1] - data[0]
        d = data[2]
        w = F/(p*d)
        rospy.loginfo("%f  %f   %f   %f   %f   %f    %f", data[0], data[1], data[2], F, p, d, w)


        var_abs_width_obj = w
        return var_abs_width_obj

if __name__ == '__main__':
    node = SizeEstimatorNode()
    
    while not rospy.is_shutdown():
        rospy.spin()



