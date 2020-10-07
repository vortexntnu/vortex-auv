#!/usr/bin/env python

import rospy
import numpy as np

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

# action message
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


class Init_state(State):
    def __init__(self):
        State.__init__(self, outcomes = ["first_state"])
    
    def execute(self, userdata):
        print("Starting state Init_state")

        return "first_state"

class Init_point_1(State):
    
    def __init__(self):
        State.__init__(self, outcomes = ["succeded"])
    
    def execute(self, userdata):
        print("Starting state init_point_1")
        return "succeded"

class Init_point_0(State):
    
    def __init__(self):
        State.__init__(self, outcomes = ["succeded"])

    
    def execute(self, userdata):
        print("Starting state init_point_0")
        return "succeded"


def make_los_goal(x_prev, y_prev, x_next, y_next, depth, speed=0.20, sphere_of_acceptance=0.5):
    
    los_goal = LosPathFollowingGoal()

    los_goal.prev_waypoint.x = x_prev
    los_goal.prev_waypoint.y = y_prev

    los_goal.next_waypoint.x = x_next
    los_goal.next_waypoint.y = y_next

    los_goal.desired_depth.z = depth

    los_goal.forward_speed.linear.x = speed
    los_goal.sphereOfAcceptance = sphere_of_acceptance
    return los_goal


class TaskManager():

    def __init__(self):

        self.psi = 0


        rospy.init_node('pathplanning_sm', anonymous=False)


        hsm = StateMachine(outcomes=['finished statemachine'])
        
        with hsm:
            StateMachine.add('INIT', Init_state(), transitions={"first_state": 'INIT_GO_TO_POINT_1'})
            StateMachine.add('INIT_GO_TO_POINT_1', Init_point_1(), transitions={ "succeded": 'GO_TO_POINT_1' })
            StateMachine.add(   'GO_TO_POINT_1',
                        SimpleActionState(  'pid_global_plan_server',
                                            LosPathFollowingAction,
                                            make_los_goal(0.0, 0.0, -10.0, 0.0, -0.5, sphere_of_acceptance=2.0)),
                                            transitions = { "succeeded": 'INIT_GO_TO_POINT_0',
                                                            "preempted": 'INIT_GO_TO_POINT_0', 
                                                            "aborted": 'INIT_GO_TO_POINT_0' })   
            StateMachine.add('INIT_GO_TO_POINT_0', Init_point_0(), transitions={ "succeded": 'GO_TO_POINT_0'})      
            StateMachine.add(   'GO_TO_POINT_0',
                         SimpleActionState( 'inspect_point_srv',
                                            LosPathFollowingAction,
                                            make_los_goal(0.0, 0.0, -10.0, 0.0, -0.5, sphere_of_acceptance=2.0)),
                                            transitions = { 'succeeded': 'INIT_GO_TO_POINT_1',
                                                            "preempted": 'INIT_GO_TO_POINT_1', 
                                                            "aborted": 'INIT_GO_TO_POINT_1' })  
            
        

        


        intro_server = IntrospectionServer(str(rospy.get_name()), hsm,'/SM_ROOT')
        intro_server.start()
        hsm.execute()
        #patrol.execute()
        print("State machine execute finished")
        intro_server.stop()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)



    

if __name__ == '__main__':
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")