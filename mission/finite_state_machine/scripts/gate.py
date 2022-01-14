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


class GateSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_search_output'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position) 
        self.gate_position = self.landmarks_client("gate").pos

        st_pub = rospy.Publisher('state_transition', String, queue_size=10)     
        st_pub.publish("GATE_SEARCH")
        
    def execute(self, userdata):
        
        while self.gate_position.x == 0: #should not be 0, but c++ NULL translates to 0.0 ...
            print("SEARCHING FOR GATE ...")
            rospy.wait_for_service('send_positions')   
            self.gate_position = self.landmarks_client("gate").pos
        
        print("GATE POSITION DETECTED: "+ str(self.gate_position.x) + ", "+ str(self.gate_position.y))                
        userdata.gate_search_output = self.gate_position       
        return 'succeeded'
    
class GateConverge(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate_position']) 
        
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position) 

        dp_guidance_action_server="/guidance_interface/dp_server" 
        self.action_client = actionlib.SimpleActionClient(dp_guidance_action_server, MoveBaseAction) 

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)
    
    def execute(self, userdata):

        # goal = MoveBaseGoal()
        # goal.target_pose.pose.position = Point(userdata.gate_position.x,userdata.gate_position.y,userdata.gate_position.z)
        # goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        # self.action_client.wait_for_server()
        # self.action_client.send_goal(goal)

        goal = VtfPathFollowingGoal()
        p = Point(userdata.gate_position.x,userdata.gate_position.y,userdata.gate_position.z)
        goal.waypoints =[p]
        goal.forward_speed = 0.1
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.action_client.cancel_all_goals()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            # goal.target_pose.pose.position = self.landmarks_client("gate").pos
            # goal.append(p)
            # self.vtf_client.cancel_goal()
            # self.vtf_client.send_goal(goal)

            rate.sleep()

        return 'succeeded'


class GateExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate_position'])           

    def execute(self, userdata):
        print("should perform the dp move to "+str(userdata.gate_position.x + 1)+" , " +str(userdata.gate_position.y))
        gate_move_through_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
        with gate_move_through_sm:
            StateMachine.add('MOVE_THROUGH_GATE_X',
                        dp_move(userdata.gate_position.x + 1,userdata.gate_position.y))
        try:
            gate_move_through_sm.execute()

        except Exception as e:
            rospy.loginfo("gate_move_sm failed: %s" % e)

        return 'succeeded'