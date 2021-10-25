#!/usr/bin/env python

import rospy

# Import msg types
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image
from vortex_msgs.msg import BBox, BBoxes
from std_msgs.msg import Int64, Float64
from geometry_msgs.msg import PointStamped, Point
import tf

# Import classes
from size_estimator import SizeEstimatorClass as SEC
from coord_pos import CoordPositionClass as CPC

class OD_NODE():
    test = 'T'
    def __init__(self):
        rospy.init_node('object_detection_node')
        self.estimatorSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.estimatorSub_callback)
        # self.imageSub = rospy.Subscriber('/darknet_ros/detection_image', Image, self.img_CB)
        self.estimatorPub = rospy.Publisher('/object_detection/size_estimates', BBoxes, queue_size= 1)
        # self.pointPub = rospy.Publisher('/object_detection/object_point', PointStamped, queue_size= 1)
        self.size_estimator = SEC()
        self.coord_positioner = CPC()

    def estimatorSub_callback(self, data):
        # Allocates msg data to local variables
        # In order to process abs size
        ArrayBoundingBoxes = BBoxes()
        ArrayBoundingBoxes.header = data.header
        ArrayBoundingBoxes.image_header = data.image_header

        # Iterate through all the detected objects and estimate sizes
        for bbox in data.bounding_boxes:

            # Store depth measurement of boundingbox
            depth_mtr = bbox.z

            # Get the size estimation from the size estimator class
            size_estimator_data = self.size_estimator.main(bbox)
            length_x_mtr = size_estimator_data[0]
            length_y_mtr = size_estimator_data[1]
            redefined_angle_x = size_estimator_data[2]
            redefined_angle_y = size_estimator_data[3]

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
            
            # This part may seem superfluos at first. The idea is that for each class of object that is detected,
            # a seperate point will be generated and published. When the camera is looking at the objects the position
            # will be updated, and when it looks somewhere else, the last know location will be stored in the message(dont think storing works).
            # Inefficient since it publishes a new topic for each class name
            # Idea is that self.test is replaced by str(bbox.Class) so the if statement will be replaced by only
            pointPub = rospy.Publisher('/object_detection/object_point/' + str(bbox.Class), PointStamped, queue_size= 1)
            # if self.test == 'T':
            #     pointPub = rospy.Publisher('/object_detection/object_point/' + self.test, PointStamped, queue_size= 1)
            #     self.test = 'P'
            # else:
            #     pointPub = rospy.Publisher('/object_detection/object_point/' + self.test, PointStamped, queue_size= 1)
            #     self.test = 'T'
            # Build the message to place detected obejct in relation to camera
            new_point = PointStamped()
            new_point.header = data.header
            new_point.header.stamp = rospy.get_rostime()

            # Get the position of the object relative to the camera
            vector = self.coord_positioner.calc_3D_vector(redefined_angle_x, redefined_angle_y, depth_mtr)
            new_point.point.x = vector[0]
            new_point.point.y = vector[1]
            new_point.point.z = vector[2]

            # Publish message for each detected object --> rethink this, is it smart to have this in the loop, or should a list of point be sendt instead.
            pointPub.publish(new_point)

        self.estimatorPub.publish(ArrayBoundingBoxes)

if __name__ == '__main__':
    node = OD_NODE()

    while not rospy.is_shutdown():
        rospy.spin()

