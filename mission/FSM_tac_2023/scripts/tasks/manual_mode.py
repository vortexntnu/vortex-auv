import rospy
import smach


class ManualMode(smach.State):
    def __init__(self, outcomes=["gripper_horisontal", "gripper_vertical"]):
        # initialization
        pass

    def execute(self, userdata):
        pass