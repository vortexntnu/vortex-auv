import rospy
from smach import StateMachine
import smach
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from fsm_helper import dp_move, los_move
from nav_msgs.msg import Odometry

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from landmarks.srv import request_position
from tf.transformations import quaternion_from_euler

class MoveToGate(smach.State):
    
    def __init__(self, odom):
        self.odom = odom
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate_position']) 
        
        dp_guidance_action_server="/guidance_interface/dp_server" 

        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position) 
        self.action_client = actionlib.SimpleActionClient(dp_guidance_action_server, MoveBaseAction) 
    
    def execute(self, userdata):

        goal = MoveBaseGoal()
        goal.target_pose.pose.position = Point(userdata.gate_position.x,userdata.gate_position.y,userdata.gate_position.z)
        goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.action_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            goal.target_pose.pose.position = self.landmarks_client("gate").pos
            self.action_client.cancel_goal()
            self.action_client.send_goal(goal)
            rate.sleep()

        return 'succeeded'