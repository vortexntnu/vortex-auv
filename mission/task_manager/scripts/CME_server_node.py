#!/usr/bin/env python3
import rospy

from dynamic_reconfigure.server import Server
from CME.cfg import BelugaFSMConfig
import test_1_node
import test_2_node


def callback(config, level):
    rospy.loginfo("""State change request: {Tac_states}""".format(**config))
    test_1: int = 0
    test_2: int = 1


    if config["Tac_states"] == test_1:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/taks_2", False)
        # Start the node here
        rospy.set_param("/tasks/task_1", True)

        param = rospy.get_param("tasks/task_1")
        rospy.loginfo("Test 1 started, ", param)
    
    if config["Tac_states"] == test_2:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/task_1", False)
        # Start the node here
        rospy.set_param("/tasks/taks_2", True)

        rospy.loginfo("Test 2 started")

    # if config["joystick_interface"]:
    #     # Stop all other nodes from last state.

    #     # Start the node here
        
    #     rospy.loginfo("")
    # if config["horisontal_gripper"]:
    #     # Stop all other nodes from last state.

    #     # Start the node here
    #     rospy.loginfo("")
    
    return config



if __name__ == "__main__":
    rospy.init_node("CME_server", anonymous=False)

    srv = Server(BelugaFSMConfig, callback)
    rospy.spin()
