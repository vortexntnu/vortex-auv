import rospy
import os
import smach
import smach_ros
from vortex_msgs.msg import PropulsionCommand, CameraObjectInfo

def listener():
    rospy.init_node("listener")
    sub = rospy.Subscriber('camera_object_info', CameraObjectInfo, queue_size=1000)

    print(sub)
    rospy.spin()
if __name__ == "__main__":
    listener()