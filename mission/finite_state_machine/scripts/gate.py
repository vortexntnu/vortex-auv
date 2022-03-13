import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position
from nav_msgs.msg import Odometry
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal, SetVelocityGoal, SetVelocityAction

from tf.transformations import quaternion_from_euler
from fsm_helper import dp_move, rotate_certain_angle
from vortex_msgs.srv import ControlMode


class GateSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_search_output'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position) 
        self.object = self.landmarks_client("gate").object_pos
        self.object_seen = self.object.isDetected
        self.gate_position = self.object.objectPose.pose.position

        self.state_pub = rospy.Publisher('/fsm/state',String,queue_size=1)


        self.drone_pose = Pose()
        rospy.Subscriber(rospy.get_param("/controllers/vtf/odometry_topic"), Odometry, self.read_position)

        vel_guidance_action_server="/guidance_interface/vel_server" 
        self.vel_action_client = actionlib.SimpleActionClient(vel_guidance_action_server, SetVelocityAction) 
        
    def execute(self, userdata):
        self.state_pub.publish("gate_search")
        rate = rospy.Rate(1)
        goal = SetVelocityGoal()
        goal.desired_velocity.linear.z = -0.00001
        goal.desired_velocity.angular.z = 0.05
        self.vel_action_client.send_goal(goal)    
        while not self.object.isDetected:

            print("SEARCHING FOR GATE ...")
            print(self.gate_position)
            rospy.wait_for_service('send_positions')   
            self.object = self.landmarks_client("gate").object_pos

            
            rate.sleep()
        goal.desired_velocity.linear.z = 0
        goal.desired_velocity.angular.z = 0
        self.vel_action_client.send_goal(goal) 
        self.vel_action_client.cancel_all_goals()
        print("GATE POSITION DETECTED: "+ str(self.gate_position.x) + ", "+ str(self.gate_position.y))                
        userdata.gate_search_output = self.gate_position       
        return 'succeeded'
    
    def read_position(self, msg):
        self.drone_pose = msg.pose.pose
    
class GateConverge(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate_position'],output_keys=['gate_converge_output']) 
        
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
        if (self.odom.pose.pose.position.x < userdata.gate_position.x):
            p = Point(userdata.gate_position.x-0.5,userdata.gate_position.y,userdata.gate_position.z)
        else:
            p = Point(userdata.gate_position.x+0.5,userdata.gate_position.y,userdata.gate_position.z)
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
            goal.waypoints = [self.landmarks_client("gate").object_pos.objectPose.pose.position]
            userdata.gate_converge_output=goal.waypoints[0]
            print("GATE POSITION DETECTED: "+ str(goal.waypoints[0].x) + ", "+ str(goal.waypoints[0].y)+ ", "+ str(goal.waypoints[0].z))
            if (self.odom.pose.pose.position.x < userdata.gate_position.x):
                goal.waypoints[0].x = goal.waypoints[0].x-0.5
            else:
                goal.waypoints[0].x = goal.waypoints[0].x+0.5
            self.vtf_client.send_goal(goal)
            rate.sleep()

        return 'succeeded'
    

class GateExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate_position'])   
        
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)        

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        goal = VtfPathFollowingGoal()
        if (self.odom.pose.pose.position.x < userdata.gate_position.x):
            p = Point(userdata.gate_position.x+1,userdata.gate_position.y,userdata.gate_position.z)
        else:
            p = Point(userdata.gate_position.x-1,userdata.gate_position.y,userdata.gate_position.z)
        goal.waypoints =[p]
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