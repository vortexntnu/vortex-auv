import rospy
import smach
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from landmarks.srv import request_position

from vortex_msgs.msg import (
    DpAction,
    DpGoal
)



class DockingSearch(smach.State):

    # TODO: Wait for docking_point from landmark server

    def __init__(self):
        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")

    def execute(self, userdata):
        pass
    

class DockingConverge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        self.landmarks_client = rospy.ServiceProxy("send_positions", request_position)
        rospy.wait_for_service("send_positions")
        self.object = self.landmarks_client("docking").object

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # TODO: name dp_action_server, remember DockingConverge aswell
        dp_action_server = ""
        self.dp_client(actionlib.SimpleActionClient(
            dp_action_server, DpAction
        ))

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("docking/converge")

        goal = DpGoal()

        goal.x_ref = self.object.pose  # WARNING: this is the centre of mass, not the landing point
        
        # activates DP for all degrees of freedom
        goal.DOF = [1,1,1,1,1,1]

        self.dp_client.wait_for_server()
        self.dp_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()

        while (
            not rospy.is_shutdown()
            and not self.dp_client.simple_state
            == actionlib.simple_action_client.SimpleGoalState.DONE
        ):
            self.object = self.landmarks_client("docking").object

            # TODO: create new pose object shifted to landing point

            goal.x_ref = self.object.pose   # WARNING: this is the centre of mass, not the landing point
            print(
                "DOCKING POINT DETECTED: "
                + str(goal.x_ref.Point.x)
                + ", "
                + str(goal.x_ref.Point.y)
                + ", "
                + str(goal.x_ref.Point.z)
            )
            self.dp_client.send_goal(goal)
            rate.sleep()

            if self.object.estimateFucked:
                self.vtf_client.cancel_all_goals()
                return "aborted"
        self.dp_client.cancel_all_goals()

        self.object = self.landmarks_client("docking").object
        print(
            "DOCKING POINT ESTIMATE CONVERGED AT: "
            + str(self.object.objectPose.pose.position.x)
            + "; "
            + str(self.object.objectPose.pose.position.y)
            + "; "
            + str(self.object.objectPose.pose.position.z)
        )
        return "succeeded"

        
class DockingExecute(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        dp_action_server = ""
        self.dp_client(actionlib.SimpleActionClient(
            dp_action_server, DpAction
        ))

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg   

    def execute(self):
        pass

        # TODO: stay docked and pull out when ready

