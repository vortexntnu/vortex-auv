#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {Joystick}, {valve_v}, {valve_h}".format(**config))


if __name__=="__main__":
    rospy.init_node("CME_client")

    CME_client = dynamic_reconfigure.client.Client("CME_client", timeout=30)
    
    while not rospy.is_shutdown():
        x=True
        y=False
        z=False

        CME_client.update_configuration({"Joystick": x,
                                        "valve_v":   y,
                                        "valve_h":   z,
                                        })
        rospy.spin()
        