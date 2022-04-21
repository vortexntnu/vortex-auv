import rospy
import smach
from smach import StateMachine, State
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

class BuoySearch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'])           
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        rospy.wait_for_service('send_positions') 
        self.object = self.landmarks_client("buoy").object

        self.state_pub = rospy.Publisher('/fsm/state',String,queue_size=1)

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)


    def execute(self, userdata):
        userdata.buoy_converge_output=self.object
        return 'succeded'


class BuoyConverge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['buoy_converge_output']) 
        
        self.landmarks_client = rospy.ServiceProxy('send_positions',request_position)
        rospy.wait_for_service('send_positions') 
        self.object = self.landmarks_client("buoy").object

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction)

        self.dp_pub = rospy.Publisher("controllers/dp_data", DpSetpoint)

    def execute(self, userdata):

        goal = VtfPathFollowingGoal()
        # self.object = self.landmarks_client("buoy").object        Should be unnecessary to re-initialize
        goal_pose = get_pose_in_front(self.object.objectPose.pose, 0.5) 
        print("get_pose_in_front returned:")
        print(goal_pose)
        goal.waypoints = [goal_pose.position]
        goal.forward_speed = 0.1
        goal.heading = "path_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown() \
            and not self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
            self.object = self.landmarks_client("buoy").object
            goal.waypoints = [self.object.objectPose.pose.position]
            print("BUOY POSITION DETECTED: "+ \
                str(goal.waypoints[0].x) + ", "+ str(goal.waypoints[0].y)+ ", "+ str(goal.waypoints[0].z))
            goal.waypoints[0] = get_pose_in_front(self.object.objectPose.pose, 0.5).position
            self.vtf_client.send_goal(goal)
            rate.sleep()
            if self.object.estimateFucked:
                self.vtf_client.cancel_all_goals()
                return 'preempted'
        self.vtf_client.cancel_all_goals()

        dp_goal = DpSetpoint()
        dp_goal.control_mode = 7 # POSE_HOLD
        dp_goal.setpoint = get_pose_in_front(self.object.objectPose.pose, 0.5)
        self.dp_pub.publish(dp_goal)
        while not rospy.is_shutdown()\
            and not self.object.estimateConverged:
            self.object = self.landmarks_client("buoy").object
            if self.object.estimateFucked:
                dp_goal.control_mode = 0 # OPEN_LOOP
                self.dp_pub.publish(dp_goal)
                return 'preempted'
            rate.sleep()
        dp_goal.control_mode = 0 # OPEN_LOOP
        self.dp_pub.publish(dp_goal)
        self.object = self.landmarks_client("buoy").object
        userdata.buoy_converge_output=self.object
        print("BUOY POSITION ESTIMATE CONVERGED AT: " + str(self.object.objectPose.position.x) + "; " \
        + str(self.object.objectPose.position.y) + "; " \
        + str(self.object.objectPose.position.z))
        return 'succeded'

class BuoyExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['buoy'])   
        
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction) 

    def execute(self, userdata):
        goal = VtfPathFollowingGoal()
        goal_pose = get_pose_in_front(userdata.buoy.objectPose.pose,-0.5 )
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

        

            