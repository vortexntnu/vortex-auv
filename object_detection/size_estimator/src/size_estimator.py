#!/usr/bin/env python


from re import X
import rospy

from darknet_ros_msgs.msg import BoundingBoxes, _BoundingBox
from sensor_msgs.msg import Image

import numpy as np
from math import sin
from std_msgs.msg import Int64, Float64


class SizeEstimatorNode():
    ppdh = 29.87
    fov_horizontal = 110.0          # Degrees
    fov_vertical = 70               # Degrees
    focal_length = 2.12             # mm
    max_width = 1344.0              # pxl
    max_height = 376.0              # pxl
    angles_pr_pxl_hor = fov_horizontal/max_width
    angles_pr_pxl_ver = fov_vertical/max_height

    def __init__(self):
        rospy.init_node('size_estimator')
        self.estimatorSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        # self.imageSub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.img_CB)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimator', Float64, queue_size= 1)

    # def img_CB(self, data):
    #     rospy.loginfo("%f  %f",data.height, data.width)


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
        # length_pxl = (data[1]-data[0])
        length_m = 0.0
        angle_max = data[1] * self.angles_pr_pxl_hor
        angle_min = data[0] * self.angles_pr_pxl_hor
        delta_angle = angle_max - angle_min
        if delta_angle >= 0:
            # h = sin(delta_angle)*length_pxl
            # conv_const = h/data[2]
            length_m = data[2] * sin(delta_angle)
        rospy.loginfo("%f  %f   %f   %f   %f   %f    %f", data[0], data[1], data[2], length_m, delta_angle, angle_max, angle_min)
        return length_m

    def abs_width_obj(self, data):
        # Do stuff
        # length_pxl = (data[1]-data[0])
        width_m = 0.0
        angle_max = data[1] * self.angles_pr_pxl_ver
        angle_min = data[0] * self.angles_pr_pxl_ver
        delta_angle = angle_max - angle_min
        if delta_angle >= 0:
            # h = sin(delta_angle)*length_pxl
            # conv_const = h/data[2]
            width_m = data[2] * sin(delta_angle)
        rospy.loginfo("%f  %f   %f   %f   %f   %f    %f", data[0], data[1], data[2], width_m, delta_angle, angle_max, angle_min)
        return width_m

if __name__ == '__main__':
    node = SizeEstimatorNode()
    
    while not rospy.is_shutdown():
        rospy.spin()



