#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose

class WaypointSubNode(object):

    def __init__(self):
        rospy.init_node('waypoint_sub_node')
        self.subscriber = rospy.Subscriber('/generated_path', Path, self.callback, queue_size = 1)

    def callback(self, msg):
         x_plt = []
         y_plt = []
         print msg.poses
         
         for i in range(msg.poses.__len__()):
             print "no of waypoints: ", msg.poses.__len__()
             x_plt.append(msg.poses[i].pose.position.x)
             print "x: ", msg.poses[i].pose.position.x
             y_plt.append(msg.poses[i].pose.position.y)
             print "y: ", msg.poses[i].pose.position.y
             
         plt.plot(x_plt, y_plt, 'ro')        
         plt.axis([-25,25,-25,25])
         plt.show()


if __name__ == '__main__':
    try:
        WP_sub_node = WaypointSubNode()
        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass


