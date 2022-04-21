import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal, SetVelocityGoal, SetVelocityAction

from tf.transformations import quaternion_from_euler
from fsm_helper import dp_move, get_pose_in_front, rotate_certain_angle
from vortex_msgs.srv import ControlMode


class PathSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        rospy.wait_for_service('send_positions') 
        self.object = self.landmarks_client("path").object

        self.state_pub = rospy.Publisher('/fsm/state',String,queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)  

        
    def execute(self, userdata):
        self.state_pub.publish("path_search")

        goal = VtfPathFollowingGoal()
        goal.waypoints = [Point(5,0,-0.5)]
        goal.forward_speed = 0.2
        goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(10)

        while not self.object.isDetected:

            print("SEARCHING FOR PATH ...")
            print(self.object.objectPose.pose.position)
            rospy.wait_for_service('send_positions')   
            self.object = self.landmarks_client("path").object
            rate.sleep()
        
        self.vtf_client.cancel_all_goals()

        print("PATH POSITION DETECTED: "+ 
            str(self.object.objectPose.pose.position.x) + ", "+ 
            str(self.object.objectPose.pose.position.y) + ", "+ 
            str(self.object.objectPose.pose.position.z))                
        return 'succeeded'
    
    
class PathConverge(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['path_converge_output']) 
        
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        rospy.wait_for_service('send_positions') 
        self.object = self.landmarks_client("path").object

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)

    def execute(self, userdata):

        goal = VtfPathFollowingGoal()
        self.object = self.landmarks_client("path").object
        goal_pose = get_pose_in_front(self.object.objectPose.pose, 0.5) 
        print("get_pose_in_front returned:")
        print(goal_pose)
        goal.waypoints =[goal_pose.position]
        goal.forward_speed = 0.1
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE and self.object.estimateConverged:
                break
            self.object = self.landmarks_client("path").object
            goal.waypoints = [self.object.objectPose.pose.position]
            print("PATH POSITION DETECTED: "+ str(goal.waypoints[0].x) + ", "+ str(goal.waypoints[0].y)+ ", "+ str(goal.waypoints[0].z))
            goal.waypoints[0] = get_pose_in_front(self.object.objectPose.pose, 0.5).position
            self.vtf_client.send_goal(goal)
            userdata.gate_converge_output=self.object
            rate.sleep()
            

        return 'succeeded'

class PathExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['path'])   
        
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)        

    def execute(self, userdata):
        goal = VtfPathFollowingGoal()
        goal_pose = get_pose_in_front(userdata.gate.objectPose.pose,-0.5)
        goal.waypoints =[goal_pose.position]
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