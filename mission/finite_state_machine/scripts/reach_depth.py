import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position

import actionlib
from actionlib_msgs.msg import GoalStatus
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal
from landmarks.srv import request_position
from tf.transformations import quaternion_from_euler
from vortex_msgs.srv import ControlMode  # , ControlModeRequest
from fsm_helper import within_acceptance_margins
from nav_msgs.msg import Odometry
from fsm_helper import dp_move, los_move
from vortex_msgs.srv import ControlMode  # , ControlModeRequest


class ReachDepth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["preempted", "succeeded", "aborted"])
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

    def execute(self, userdata):
        goal = VtfPathFollowingGoal()
        goal.waypoints = [Point(0.1, 0, -1)]
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (
                self.vtf_client.simple_state
                == actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                print("ReachDepth succeeded")
                break
            rate.sleep()

        return "succeeded"
