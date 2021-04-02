import rospy
import smach
from std_msgs.msg import Bool


class ReachStartPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["doing", "continue"])
        # subscribe to signal
        print("Init")
        rospy.Subscriber("condition_topic", Bool, self.condition_callback)
        self.rate = rospy.Rate(10)
        self.condition_reached = False

    def condition_callback(self, condition_signal):
        if condition_signal.data == True:
            self.condition_reached = not self.condition_reached

    def execute(self, userdata):
        if self.condition_signal:
            return "continue"

        self.rate.sleep()
        return "waiting"
