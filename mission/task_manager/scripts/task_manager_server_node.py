#!/usr/bin/python3
import rospy

from dynamic_reconfigure.server import Server
from task_manager.cfg import BelugaFSMConfig

from task_manager_defines import defines


def callback(config, level):
    rospy.loginfo("""State change request: {Tac_states}""".format(**config))
    active_task_id = config["Tac_states"]

    for task in defines.Tasks.tasks:
        rospy.set_param(f"/tasks/{task.name}", False)

        # param = rospy.get_param(f"/tasks/{task.name}")
        # rospy.loginfo(f"Inactive task name: {task.name}, true/false: {param}")

        if task.id == active_task_id:
            rospy.set_param(f"/tasks/{task.name}", True)

            # param = rospy.get_param(f"/tasks/{task.name}")
            # rospy.loginfo(f"Active task name: {task.name}, true/false: {param}")

    return config


if __name__ == "__main__":
    rospy.init_node("task_manager_server", anonymous=False)

    srv = Server(BelugaFSMConfig, callback)
    rospy.spin()
