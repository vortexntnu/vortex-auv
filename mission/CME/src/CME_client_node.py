#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client




if __name__=="__main__":
    rospy.init_node("CME_client")

    CME_client = dynamic_reconfigure.client.Client("CME_client", timeout=30)
    
    while not rospy.is_shutdown():
        CME_client.update_configuration()
        