import rospy
import smach


class TestModeTwo(smach.State):
    # initialization
    def __init__(self):
        self.task = "test_mode_two"
        smach.State.__init__(self, outcomes=["outcome_2"])

    # Execution of the state
    def execute(self, userdata):
        rospy.loginfo("Executing state TestModeTwo")
        return "outcome_2"