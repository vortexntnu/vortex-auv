import rospy
import smach


class DockingSearch(smach.State):
    def __init__(self):
        pass

    def execute(self, userdata):
        pass
    

class DockingConverge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("docking").object

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("docking/converge")

        
class DockingExecute(smach.State):
    def __init__(self):
        pass

    def execute(self, userdata):
        pass

