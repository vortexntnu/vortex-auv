import rospy
import smach
from std_msgs.msg import Bool


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["doing", "waiting"])
        # subscribe to signal
        print("Init")
        rospy.Subscriber("mission_trigger", Bool, self.mission_trigger_callback)
        self.rate = rospy.Rate(10)
        self.mission_in_progress = False

    def mission_trigger_callback(self, trigger_signal):
        if trigger_signal.data == True:
            self.mission_in_progress = not self.mission_in_progress

    def execute(self, userdata):
        if self.mission_in_progress:
            return "doing"

        self.rate.sleep()
        return "waiting"
