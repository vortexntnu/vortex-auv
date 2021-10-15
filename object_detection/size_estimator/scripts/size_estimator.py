#!/usr/bin/env python

import rospy

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image
from vortex_msgs.msg import BBox, BBoxes

import numpy as np
from math import sin
from std_msgs.msg import Int64, Float64


class SizeEstimatorNode():
    fov_horizontal = 110.0                          # Degrees
    fov_vertical = 70                               # Degrees
    focal_length = 2.12                             # mm
    max_width = 1344.0                              # pxl
    max_height = 376.0                              # pxl
    angles_pr_pxl_hor = fov_horizontal/max_width
    angles_pr_pxl_ver = fov_vertical/max_height

    def __init__(self):
        rospy.init_node('size_estimator')
        self.estimatorSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        # self.imageSub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.img_CB)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimator', BBoxes, queue_size= 1)


    def callback(self, data):
        # Allocates msg data to local variables
        # In order to process abs size

        ArrayBoundingBoxes = BBoxes()
        ArrayBoundingBoxes.header = data.header
        ArrayBoundingBoxes.image_header = data.image_header

        # Iterate through all the detected objects and estimate sizes
        for bbox in data.bounding_boxes:

            # Get x-pos of boundingbox
            min_x_pxl = bbox.xmin
            max_x_pxl = bbox.xmax

            # Get y-pos of boundingbox
            min_y_pxl = bbox.ymin
            max_y_pxl = bbox.ymax
            
            # Get depth measurement of boundingbox
            depth_mtr = bbox.z

            # Calculate the angles for x and y
            delta_angle_x, angle_centre_object_x = self.calc_angles(self.angles_pr_pxl_hor, min_x_pxl,max_x_pxl)
            delta_angle_y, angle_centre_object_y = self.calc_angles(self.angles_pr_pxl_ver, min_y_pxl,max_y_pxl)

            # Calculate the sizes in metres of a boundingbox
            length_x_mtr = self.calc_size(delta_angle_x, depth_mtr)
            length_y_mtr = self.calc_size(delta_angle_y, depth_mtr)
            

            # Build the new bounding box message
            CurrentBoundingBox = BBox()
            CurrentBoundingBox.Class = bbox.Class
            CurrentBoundingBox.probability = bbox.probability
            CurrentBoundingBox.width = length_x_mtr
            CurrentBoundingBox.height = length_y_mtr
            CurrentBoundingBox.z = depth_mtr
            CurrentBoundingBox.centre_angle_x = angle_centre_object_x
            CurrentBoundingBox.centre_angle_y = angle_centre_object_y

            # Append the new message to bounding boxes array
            ArrayBoundingBoxes.bounding_boxes.append(CurrentBoundingBox)

        self.estimatorPub.publish(ArrayBoundingBoxes)

    def calc_size(self, delta_angle, dist_to_object):
        """
        Calculate angles of bounding box width/height.
          
        Args:
            delta_angle             The angle between max/min position
            dist_to_object          The distance to the object in the bounding box
        Returns:
            length_in_mtr           The width or height of the detected object box
        """

        length_in_mtr = dist_to_object * sin(delta_angle)
        return length_in_mtr

    def calc_angles(self, angles_pr_pxl, min_pos, max_pos):
        """
        Calculate angles of bounding box width/height.
          
        Args:
            angles_pr_pxl   angles_pr_pxl_hor/angles_pr_pxl_ver
            min_pos         minimum pxl position to calculate angle
            max_pos         maximum pxl position to calculate angle

        Returns:
            delta_angle             The angle between max/min position
            angle_centre_object     The angle to the centre of the object in 1 dimension
        """

        angle_max = max_pos * angles_pr_pxl
        angle_min = min_pos * angles_pr_pxl
        delta_angle = angle_max - angle_min
        angle_centre_object = angle_min + delta_angle*0.5
        
        return delta_angle, angle_centre_object



if __name__ == '__main__':
    node = SizeEstimatorNode()
    
    while not rospy.is_shutdown():
        rospy.spin()



