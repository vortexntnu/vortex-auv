#!/usr/bin/python3

import rospy
import dynamic_reconfigure.client
from task_manager_defines import defines

class ValveVertical:
    def __init__(self):
        rospy.init_node("valve_vertical")

        self.isEnabled = False


    def callback(self, config):
        rospy.loginfo("""Client: state change request: {Tac_states}""".format(**config))
        activated_task_id = config["Tac_states"]

        if defines.Tasks.valve_vertical.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"isEnabled: {self.isEnabled} ")

        return config



if __name__ == "__main__":
    valve_vertical = ValveVertical()
    task_manager_client = dynamic_reconfigure.client.Client(
    "task_manager/task_manager_server", timeout=5, config_callback=valve_vertical.callback)
    rospy.spin()
