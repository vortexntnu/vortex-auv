#!/usr/bin/env python3
import rospy

from dynamic_reconfigure.server import Server
from CME.cfg import BelugaFSMConfig


def callback(config, level):
    rospy.loginfo("""State change request: {Tac_states}""".format(**config))
    print(config)
    return config


if __name__ == "__main__":
    rospy.init_node("CME_server", anonymous=False)

    srv = Server(BelugaFSMConfig, callback)
    rospy.spin()
