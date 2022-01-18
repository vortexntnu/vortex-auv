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
from fsm_helper import dp_move, los_move, create_circle_coordinates
from vortex_msgs.srv import ControlMode
from nav_msgs.msg import Odometry

class PoleSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['pole_search_output'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        self.pole_position = self.landmarks_client("pole").pos
        
    def execute(self, userdata):

        while self.pole_position.x == 0:
            print("SEARCHING FOR POLE ...")
            rospy.wait_for_service('send_positions')   
            self.pole_position = self.landmarks_client("pole").pos
        
        print("POLE POSITION DETECTED: "+ str(self.pole_position.x) + ", "+ str(self.pole_position.y)+ ", "+ str(self.pole_position.z))    
                        
        userdata.pole_search_output = self.pole_position       
        return 'succeeded'

class PoleConverge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['pole_position'],output_keys=['pole_converge_output'])
        
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)  
        
        dp_guidance_action_server="/guidance_interface/dp_server" 
        self.action_client = actionlib.SimpleActionClient(dp_guidance_action_server, MoveBaseAction)   
 
        dp_controller_control_mode_service = "/guidance/dp_guidance/controlmode_service"
        self.control_mode_client = rospy.ServiceProxy(dp_controller_control_mode_service, ControlMode)  

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction) 

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg     

    def execute(self, userdata):
        goal = VtfPathFollowingGoal()
        if (self.odom.pose.pose.position.x < userdata.pole_position.x):
            p = Point(userdata.pole_position.x-0.5,userdata.pole_position.y,userdata.pole_position.z)
        else:
            p = Point(userdata.pole_position.x+0.5,userdata.pole_position.y,userdata.pole_position.z)
        goal.waypoints =[p]
        goal.forward_speed = 0.1
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.action_client.cancel_all_goals()
        self.control_mode_client(0)
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            goal.waypoints = [self.landmarks_client("pole").pos]
            print("POLE POSITION DETECTED: "+ str(goal.waypoints[0].x) + ", "+ str(goal.waypoints[0].y)+ ", "+ str(goal.waypoints[0].z))
            if (self.odom.pose.pose.position.x < userdata.pole_position.x):
                goal.waypoints[0].x = goal.waypoints[0].x-0.5
            else:
                goal.waypoints[0].x = goal.waypoints[0].x+0.5
            self.vtf_client.send_goal(goal)
            rate.sleep()
        userdata.pole_converge_output=self.landmarks_client("pole").pos
        return 'succeeded'

class PoleExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['pole_position'])  
        
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction) 

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg                

    def execute(self, userdata):
        goal = VtfPathFollowingGoal()
        start = self.odom.pose.pose.position
        
        centre = Point(userdata.pole_position.x,userdata.pole_position.y,userdata.pole_position.z)
        goal.waypoints = create_circle_coordinates(start,centre,330)
        goal.forward_speed = 0.1
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            rate.sleep()
        return 'succeeded'