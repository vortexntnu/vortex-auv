#!/usr/bin/env python3

import rospy
import smach
from manipulators.gripper_interface.scripts.auto_gripper_interface_node import GripperInterface


class ValveSearch(smach.State):
    def __init__(self):
        self.task = ""

        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

class ValveConverge(smach.State):
    def __init__(self):
        self.task = ""

        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

class ValveExecute(smach.State):
    def __init__(self):
        self.task = ""

        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        gripper = GripperInterface()

        gripper.CloseGripper()



