import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from landmarks.srv import request_position
from tf.transformations import quaternion_from_euler
from fsm_helper import dp_move, los_move

class PoleSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['pole_search_output'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        self.pole_position = self.landmarks_client("pole").pos

        st_pub = rospy.Publisher('state_transition', String, queue_size=10)     
        st_pub.publish("POLE_SEARCH")
        
    def execute(self, userdata):

        while self.pole_position.x == 0:
            print("SEARCHING FOR POLE ...")
            rospy.wait_for_service('send_positions')   
            self.pole_position = self.landmarks_client("pole").pos
        
        print("POLE POSITION DETECTED: "+ str(self.pole_position.x) + ", "+ str(self.pole_position.y))    
                        
        userdata.pole_search_output = self.pole_position       
        return 'succeeded'

class PoleConverge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['pole_position'])
        
        dp_guidance_action_server="/guidance_interface/dp_server" 
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position) 
        self.action_client = actionlib.SimpleActionClient(dp_guidance_action_server, MoveBaseAction)            

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position = Point(userdata.pole_position.x,userdata.pole_position.y,userdata.pole_position.z)
        goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.action_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            goal.target_pose.pose.position = self.landmarks_client("pole").pos
            self.action_client.cancel_goal()
            self.action_client.send_goal(goal)
            rate.sleep()
        
        return 'succeeded'

class PoleExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['pole_position'])           

    def execute(self, userdata):
        print("should perform manevour around pole ...")
        pole_move_around_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
        with pole_move_around_sm:
            StateMachine.add('MOVE_AROUND_POLE',
                        dp_move(userdata.pole_position.x,userdata.pole_position.y-1))
        
        try:
            pole_move_around_sm.execute()
        except Exception as e:
            rospy.loginfo("pole_move_around_sm failed: %s" % e)

        return 'succeeded'