#!/usr/bin/python
import numpy as np
import cv2 as cv2
import rospy
from nav_msgs.msg import OccupancyGrid
from rospkg import RosPack



class MapPublisher:
    def __init__(self):

        ros_pack = RosPack()
        path_util = ros_pack.get_path('utilities')

        pic_name = "test.jpg"
        abs_path = path_util + "/src/" + pic_name

        img = cv2.imread(abs_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print("Could not open file")

        img_shape = img.shape
        self.map_msg = self.initiateMapMsg()

        img_flatten = img.flatten()
        print("flat:")
        
        for i in range(len(img_flatten)):
            if img_flatten[i] == 0:
                self.map_msg.data[i] = 0
            else:
                self.map_msg.data[i] = 100

        self.map_pub_handle = rospy.Publisher('map', OccupancyGrid, queue_size=10)



        rospy.Timer(rospy.Duration(1.0/1.0),self.publishMapCallback)

        

    def initiateMapMsg(self):# written by Ambjorn Waldum
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'manta/odom'
        resolution = 0.1
        width = 500
        height = 500
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.data = np.arange(width*height, dtype=np.int8)
        map_msg.info.origin.position.x = - width // 2 * resolution
        map_msg.info.origin.position.y = - height // 2 * resolution
        map_msg.header.stamp = rospy.Time.now()
        return map_msg

    def publishMapCallback(self, data):
        #print("Publishing map")
        self.map_pub_handle.publish(self.map_msg)




#### INIT of node ##################################
if __name__ == '__main__':
    rospy.init_node('pre_map_publisher',anonymous=True)
    MapPublisher()
    rospy.spin()
###################################################
