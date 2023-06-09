#! /usr/bin/env python3

import rospy

import dynamic_reconfigure.client


class TaskManagerClient:
    """
    Task Manager Client class

    This class handles the integration with the task manager package.
    """

    def __init__(self, task_id):
        """
        Initialize TaskManagerClient object.
        """
        self.was_enabled = False
        self.is_enabled = False
        self.task_id = task_id

        try:
            self.task_manager_client = dynamic_reconfigure.client.Client(
                "/task_manager/task_manager_server",
                timeout=1,
                config_callback=self.callback)
        except rospy.exceptions.ROSException:
            rospy.logwarn("Could not connect to the task manager...")

    def callback(self, config):
        activated_task_id = config["Tac_states"]

        self.was_enabled = self.is_enabled
        if self.task_id == activated_task_id:
            self.is_enabled = True
        else:
            self.is_enabled = False

        return config
