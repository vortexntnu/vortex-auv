#!/usr/bin/python3
import rospy

from dynamic_reconfigure.server import Server
from task_manager.cfg import BelugaFSMConfig

from task_manager_defines import defines

import test_1_node
import test_2_node


def callback(config, level):
    rospy.loginfo("""State change request: {Tac_states}""".format(**config))

    if config["Tac_states"] == defines.Tasks.test_1.id:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/task_2", False)
        rospy.set_param("/tasks/joystick", False)
        rospy.set_param("/tasks/valve_vertical", False)
        rospy.set_param("/tasks/valve_horisontal", False)
        rospy.set_param("/tasks/pipeline_inspection", False)
        # Start the node here
        rospy.set_param("/tasks/task_1", True)

        param = rospy.get_param("/tasks/task_1")
        rospy.loginfo("Test 1 started, %s", param)

    if config["Tac_states"] == defines.Tasks.test_2.id:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/task_1", False)
        rospy.set_param("/tasks/joystick", False)
        rospy.set_param("/tasks/valve_vertical", False)
        rospy.set_param("/tasks/valve_horisontal", False)
        rospy.set_param("/tasks/pipeline_inspection", False)
        # Start the node here
        rospy.set_param("/tasks/task_2", True)

        param = rospy.get_param("/tasks/task_2")
        rospy.loginfo("Test 2 started, %s", param)

    if config["Tac_states"] == defines.Tasks.joystick:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/task_1", False)
        rospy.set_param("/tasks/task_2", False)
        rospy.set_param("/tasks/valve_vertical", False)
        rospy.set_param("/tasks/valve_horisontal", False)
        rospy.set_param("/tasks/pipeline_inspection", False)
        # Start the node here
        rospy.set_param("/tasks/joystick", True)

        param = rospy.get_param("/tasks/joystick")
        rospy.loginfo("joystick mode started, %s", param)

    if config["Tac_states"] == defines.Tasks.valve_vertical:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/task_1", False)
        rospy.set_param("/tasks/task_2", False)
        rospy.set_param("/tasks/joystick", False)
        rospy.set_param("/tasks/valve_horisontal", False)
        rospy.set_param("/tasks/pipeline_inspection", False)
        # Start the node here
        rospy.set_param("/tasks/valve_vertical", True)

        param = rospy.get_param("/tasks/valve_vertical")
        rospy.loginfo("valve_vertical mode started, %s", param)

    if config["Tac_states"] == defines.Tasks.valve_horisontal:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/task_1", False)
        rospy.set_param("/tasks/task_2", False)
        rospy.set_param("/tasks/joystick", False)
        rospy.set_param("/tasks/valve_vertical", False)
        rospy.set_param("/tasks/pipeline_inspection", False)
        # Start the node here
        rospy.set_param("/tasks/valve_horisontal", True)

        param = rospy.get_param("/tasks/valve_horisontal")
        rospy.loginfo("valve_horisontal mode started, %s", param)

    if config["Tac_states"] == defines.Tasks.pipeline_inspection:
        # Stop all other nodes from last state.
        rospy.set_param("/tasks/task_1", False)
        rospy.set_param("/tasks/task_2", False)
        rospy.set_param("/tasks/joystick", False)
        rospy.set_param("/tasks/valve_vertical", False)
        rospy.set_param("/tasks/valve_horisontal", False)
        # Start the node here
        rospy.set_param("/tasks/pipeline_inspection", True)

        param = rospy.get_param("/tasks/pipeline_inspection")
        rospy.loginfo("pipeline_inspection started, %s", param)

    return config


if __name__ == "__main__":
    rospy.init_node("task_manager_server", anonymous=False)

    srv = Server(BelugaFSMConfig, callback)
    rospy.spin()
