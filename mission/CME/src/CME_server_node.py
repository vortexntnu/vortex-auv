#!/usr/bin/python3
import rospy

from dynamic_reconfigure.server import Server
from CME.cfg import BelugaFSMConfig

def callback(config, level):
    rospy.loginfo("""State change request: {Joystick}, {valve_v}, {valve_h}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("CME_server", anonymous=True)

    srv = Server(BelugaFSMConfig, callback)
    rospy.spin()

    