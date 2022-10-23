#!/usr/bin/python3
#To be put in the Raspberrry pi
import time
import traceback

import rospy
import smbus
from std_msgs.msg import String


class XavierIPSubscriber:
    def __init__(self):
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber("xavier_IP_monitor", String, self.callback)

    def callback(self, data):
        f = open("xavier_IP.txt", "w")
        f.write(data)
        f.close()

if __name__ == "__main__":
    xi = XavierIPSubscriber()
    try:    
        rospy.spin()
    finally:
        xi.shutdown()