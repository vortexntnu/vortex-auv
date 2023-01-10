import rospy
import smach


class ValveSearch(smach.State):
    def __init__(self):
        self.task = ""

        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

class ValveConverge(smach.State):
    def __init__(self):
        self.task = ""

        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])