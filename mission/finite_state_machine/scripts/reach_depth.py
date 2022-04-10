import rospy
import smach
from smach import StateMachine
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from landmarks.srv import request_position

import actionlib
from actionlib_msgs.msg import GoalStatus
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal, DpSetpoint
from landmarks.srv import request_position
from tf.transformations import quaternion_from_euler
from fsm_helper import dp_move, los_move
from vortex_msgs.srv import ControlMode #, ControlModeRequest
from dp_helper import within_acceptance_margins
from nav_msgs.msg import Odometry

class ReachDepth(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'])
        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server, VtfPathFollowingAction) 

        self.dp_pub = rospy.Publisher("/controllers/dp_data", DpSetpoint, queue_size=1)
        
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg   
   

    def execute(self, userdata):
        print("starting DP move")
        dp_goal = DpSetpoint()
        dp_goal.control_mode = 1 #Position hold
        dp_goal.setpoint.position = Point(1,0,-0.5)
        self.dp_pub.publish(dp_goal)
        
        rate = rospy.Rate(10)
        while not within_acceptance_margins(dp_goal.setpoint,self.odom):
            rate.sleep()
        dp_goal.control_mode = 0 #Open loop
        dp_goal.setpoint.position = Point(0,0,0)
        self.dp_pub.publish(dp_goal)
        print("finished with DP move")

        goal = VtfPathFollowingGoal()
        goal.waypoints = [Point(0,0,-0.5)]
        goal.forward_speed = 0.2
        goal.heading = "path_dependent_heading"
        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.vtf_client.simple_state == actionlib.simple_action_client.SimpleGoalState.DONE:
                break
            rate.sleep()
        
        return 'succeeded'