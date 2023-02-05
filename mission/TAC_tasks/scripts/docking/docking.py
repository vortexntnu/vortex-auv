import rospy
import smach
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from landmarks.srv import request_position

from vortex_msgs.msg import (
    
)



class DockingSearch(smach.State):

    # TODO: Wait for docking_point from landmark server

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

        dp_action_server = ""
        self.dp_client(actionlib.SimpleActionClient(
            dp_action_server, 
        ))

        
class DockingExecute(smach.State):

    # TODO: stay docked and pull out when ready

    def __init__(self):
        pass

    def execute(self, userdata):
        pass

