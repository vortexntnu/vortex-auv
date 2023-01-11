import rospy
import smach

from std_msgs.msg import String
from joystick_interface import JoystickInterface

#Subscriber that listens for information about next state.
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Next State %s", data.data)
    
def listener():
    rospy.init_node("listner", anonymous=True)
    rospy.Subscriber("chatter", String, callback)


class ManualMode(smach.State):
    # initialization
    def __init__(self):
        self.task = "manual_mode"
        smach.State.__init__(self, outcomes=["valve_h", "valve_v"])

    # Execution of the state ManualMode.
    def execute(self, userdata):
        rospy.loginfo("Executing state ManualMode")

        try:
            joystick_interface = JoystickInterface()

            while not rospy.core.is_shutdown():
                if listener()=="valve_h":
                    return "valve_h"
                elif listener()=="valve_v":
                    return "valve_v"
                    
                rospy.rostime.wallsleep(0.5)

        except rospy.ROSInterruptException:
            pass
