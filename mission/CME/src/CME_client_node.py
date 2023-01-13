#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client


def callback(config):
    rospy.loginfo("Config set to {Tac_states}".format(**config))


if __name__ == "__main__":
    rospy.init_node("CME_client")

    CME_client = dynamic_reconfigure.client.Client(
        "CME_client", timeout=30, config_callback=callback
    )

    while not rospy.is_shutdown():
        rospy.spin()
