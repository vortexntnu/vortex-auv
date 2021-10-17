#!/usr/bin/env python

import rospy

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image
from vortex_msgs.msg import BBox, BBoxes

from geometry_msgs.msg import PointStamped, Point
import tf

import numpy as np
from math import cos, sin, radians
from std_msgs.msg import Int64, Float64


class SizeEstimatorNode():
    fov_horizontal = 110.0                          # Degrees
    fov_vertical = 70.0                             # Degrees
    focal_length = 2.12                             # mm
    max_width = 672                                 # pxl
    max_height = 376                                # pxl
    angles_pr_pxl_hor = fov_horizontal/max_width
    angles_pr_pxl_ver = fov_vertical/max_height

    def __init__(self):
        rospy.init_node('size_estimator')
        self.estimatorSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback)
        # self.imageSub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.img_CB)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimator', BBoxes, queue_size= 1)
        self.pointPub = rospy.Publisher('/object_detection/obj_vector', PointStamped, queue_size= 1)


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

            # Redefine angles to coord frame
            redefined_angle_x = self.redefine_angles(angle_centre_object_x, self.fov_horizontal)
            redefined_angle_y = self.redefine_angles(angle_centre_object_y, self.fov_vertical)

            # Calculate the sizes in metres of a boundingbox
            length_x_mtr = self.calc_size(delta_angle_x, depth_mtr)
            length_y_mtr = self.calc_size(delta_angle_y, depth_mtr)

            vector = self.calc_3D_vector(redefined_angle_x, redefined_angle_y, depth_mtr)
            rospy.loginfo("%f   %f   %f", vector[0], vector[1], vector[2])

            new_point = PointStamped()
            new_point.header = data.header
            new_point.header.stamp = rospy.get_rostime()

            new_point.point.x = vector[0]
            new_point.point.y = vector[1]
            new_point.point.z = vector[2]
            self.pointPub.publish(new_point)

            # Build the new bounding box message
            CurrentBoundingBox = BBox()
            CurrentBoundingBox.Class = bbox.Class
            CurrentBoundingBox.probability = bbox.probability
            CurrentBoundingBox.width = length_x_mtr
            CurrentBoundingBox.height = length_y_mtr
            CurrentBoundingBox.z = depth_mtr
            CurrentBoundingBox.centre_angle_x = redefined_angle_x
            CurrentBoundingBox.centre_angle_y = redefined_angle_y

            # Append the new message to bounding boxes array
            ArrayBoundingBoxes.bounding_boxes.append(CurrentBoundingBox)

        self.estimatorPub.publish(ArrayBoundingBoxes)
    
    def redefine_angles(self, angle_centre_object, fov_type):
        """
        Creates a coordinate system for the view. Angles in the right half of the view are positive, and the left half are negative.
        Does the same for top and bottom half of screen. Sets Origo in the center of the screen.

        Args:
            angle_centre_object         angle_centre_object_x/angle_centre_object_y
            fov_type                    fov_horizontal/fov_vertical

        Returns:
            redefined_angle             angle in new coordinate system, pos angle in right/upper half os the screen and vice versa
        """

        # Split the screen in two halves
        new_fov = fov_type * 0.5
        
        # For some reason the horizontal angle is calculated from left to right,
        # however the vertical angle is quite unintuitively calculated from top to bottom,
        # and therefore needs to be inverted to have neg degrees in the bottom half  
        if new_fov == 35:
            redefined_angle = -(angle_centre_object - new_fov)
            # rospy.loginfo("\n%f \n%f",angle_centre_object, new_fov)
        else:
            redefined_angle = angle_centre_object - new_fov

        return redefined_angle
        

    def calc_size(self, delta_angle, dist_to_object):
        """
        Calculate angles of bounding box width/height.
          
        Args:
            delta_angle             The angle between max/min position
            dist_to_object          The distance to the object in the bounding box
        Returns:
            length_in_mtr           The width or height of the detected object box
        """

        length_in_mtr = dist_to_object * sin(radians(delta_angle))

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
        angle_centre_object = angle_min + delta_angle * 0.5
        
        return delta_angle, angle_centre_object
    
    def calc_3D_vector(self, x_angle, y_angle, length):
        x_comp = length * cos(radians(x_angle)) * sin(radians(y_angle))
        y_comp = length * cos(radians(x_angle)) * cos(radians(y_angle))
        z_comp = length * sin(radians(x_angle))

        return [x_comp, y_comp, z_comp]


if __name__ == '__main__':
    node = SizeEstimatorNode()

    while not rospy.is_shutdown():
        rospy.spin()



