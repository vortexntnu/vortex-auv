import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal
from landmarks.srv import request_position
from tf.transformations import quaternion_from_euler
from fsm_helper import dp_move, los_move
from vortex_msgs.srv import ControlMode#, ControlModeRequest

class ReachDepth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'])
        dp_guidance_action_server="/guidance/dp_action_server" 
        self.action_client = actionlib.SimpleActionClient(dp_guidance_action_server, MoveBaseAction) 
        dp_controller_control_mode_service = "/guidance/dp_guidance/controlmode_service"
        self.control_mode_client = rospy.ServiceProxy(dp_controller_control_mode_service, ControlMode)      

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position = Point(0,0,0)
        goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal)
        rate = rospy.Rate(10)
        rate.sleep()
        while not rospy.is_shutdown():
            print(self.action_client.simple_state)
            if self.action_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            rate.sleep()
        self.action_client.cancel_all_goals()
        self.control_mode_client(0)
        
        return 'succeeded'