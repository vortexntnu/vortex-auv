import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal, SetVelocityGoal, SetVelocityAction, DpSetpoint

from tf.transformations import quaternion_from_euler
from fsm_helper import dp_move, get_pose_in_front, rotate_certain_angle
from vortex_msgs.srv import ControlMode


class GateSearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        rospy.wait_for_service('send_positions') 
        self.object = self.landmarks_client("gate").object

        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)

        self.state_pub = rospy.Publisher('/fsm/state',String,queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)  

        
    def execute(self, userdata):
        self.state_pub.publish("gate_search")


        #SEARCH PATTERN:
        goal = VtfPathFollowingGoal()
        goal.waypoints = [Point(1,0,-0.5)]
        goal.forward_speed = 0.2
        goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        self.vtf_client.wait_for_result()

        rate = rospy.Rate(10)

        while not self.object.isDetected:

            print("SEARCHING FOR GATE ...")
            print(self.object.objectPose.pose.position)
            rospy.wait_for_service('send_positions')   
            self.object = self.landmarks_client("gate").object
            rate.sleep()
        
        self.vtf_client.cancel_all_goals()

        print("GATE POSITION DETECTED: "+ 
            str(self.object.objectPose.pose.position.x) + ", "+ 
            str(self.object.objectPose.pose.position.y) + ", "+ 
            str(self.object.objectPose.pose.position.z))                
        return 'succeeded'
    


#TODO: incorporate DP controller to hold pose while estimate converges 
class GateConverge(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_converge_output']) 
        
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        rospy.wait_for_service('send_positions') 
        self.object = self.landmarks_client("gate").object

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)

    def execute(self, userdata):

        goal = VtfPathFollowingGoal()
        self.object = self.landmarks_client("gate").object
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
            self.object = self.landmarks_client("gate").object
            goal.waypoints = [self.object.objectPose.pose.position]
            print("GATE POSITION DETECTED: "+ str(goal.waypoints[0].x) + ", "+ str(goal.waypoints[0].y)+ ", "+ str(goal.waypoints[0].z))
            goal.waypoints[0] = get_pose_in_front(self.object.objectPose.pose, 0.5).position
            self.vtf_client.send_goal(goal)
            userdata.gate_converge_output=self.object
            rate.sleep()

            if self.object.estimateFucked:
                self.vtf_client.cancel_all_goals()
                return 'aborted'
        self.vtf_client.cancel_all_goals()

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 7 # POSE_HOLD
        dp_goal.setpoint = get_pose_in_front(self.object.objectPose.pose, 0.5)
        self.dp_pub.publish(dp_goal)
        while not rospy.is_shutdown()\
            and not self.object.estimateConverged:
            self.object = self.landmarks_client("gate").object
            if self.object.estimateFucked:
                dp_goal.control_mode = 0 # OPEN_LOOP
                self.dp_pub.publish(dp_goal)
                return 'aborted'
            rate.sleep()
        dp_goal.control_mode = 0 # OPEN_LOOP
        self.dp_pub.publish(dp_goal)
        self.object = self.landmarks_client("gate").object
        userdata.buoy_converge_output=self.object
        print("GATE POSITION ESTIMATE CONVERGED AT: " + str(self.object.objectPose.position.x) + "; " \
        + str(self.object.objectPose.position.y) + "; " \
        + str(self.object.objectPose.position.z))    
            

        return 'succeeded'

class GateExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['gate'])   
        
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