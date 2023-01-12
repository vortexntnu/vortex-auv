import rospy
import smach


class TestModeOne(smach.State):
    # initialization
    def __init__(self):
        self.task = "test_mode_one"
        smach.State.__init__(self, outcomes=["outcome_1"])

    # Execution of the state
    def execute(self, userdata):
        rospy.loginfo("Executing state TestModeOne")
        return "outcome_1"