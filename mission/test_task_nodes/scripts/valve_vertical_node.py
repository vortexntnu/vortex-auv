#!/usr/bin/python3

import rospy
import dynamic_reconfigure.client
from task_manager_defines import defines

class ValveVertical:
    def __init__(self):
        rospy.init_node("valve_vertical")

        self.isEnabled = False
        self.task_manager_client = dynamic_reconfigure.client.Client(
        "/task_manager_server", timeout=3, config_callback=self.callback)


    def callback(config, self):
        activated_task_id = config["Tac_states"]
        for task in defines.Tasks.tasks_list:

        # param = rospy.get_param(f"/tasks/{task.name}")
        # rospy.loginfo(f"Inactive task name: {task.name}, true/false: {param}")
            if task.id == activated_task_id:
                self.isEnabled = True

            # param = rospy.get_param(f"/tasks/{task.name}")
            # rospy.loginfo(f"Active task name: {task.name}, true/false: {param}")

        return config
    

    def spin(self):
        while not rospy.is_shutdown:
            rospy.spin()


if __name__ == "__main__":
    valve_vertical_task = ValveVertical()
    valve_vertical_task.spin()
