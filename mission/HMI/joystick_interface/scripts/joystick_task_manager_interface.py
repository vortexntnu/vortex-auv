#! /usr/bin/env python3

import rospy

import dynamic_reconfigure.client
from task_manager_defines import defines


class JoystickEnable:

    def __init__(self):
        rospy.init_node("JoystickEnable")

        self.isEnabled = False

        task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=5,
            config_callback=self.callback)

    def callback(self, config):
        rospy.loginfo(
            """Client: state change request: {Tac_states}""".format(**config))
        activated_task_id = config["Tac_states"]

        if defines.Tasks.joystick.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"isEnabled: {self.isEnabled} ")

        return config


if __name__ == "__main__":
    joystickEnable = JoystickEnable()
    if joystickEnable.isEnabled == True:
        try:
            joystick_interface = JoystickInterface()
            rospy.spin()

        except rospy.ROSInterruptException:
            pass
