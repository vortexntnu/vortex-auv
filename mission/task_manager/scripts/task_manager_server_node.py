#!/usr/bin/python3
import rospy

from dynamic_reconfigure.server import Server
from task_manager.cfg import TacStatesConfig


def callback(config, level):
    rospy.loginfo(
        """task manager server: state change request: {Tac_states}""".format(**config)
    )
    return config


if __name__ == "__main__":
    rospy.init_node("task_manager_server", anonymous=False)

    srv = Server(TacStatesConfig, callback)
    rospy.spin()
