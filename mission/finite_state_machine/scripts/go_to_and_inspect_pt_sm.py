#!/usr/bin/python3

import rospy
import numpy as np

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

# action message
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from vortex_msgs.msg import MoveAction, MoveGoal



def makeMoveGoal(contr_name, target_x, target_y, target_z, radius_of_acceptance = 0.2):


    """
    string controller_name
    geometry_msgs/Pose target_pose
    float32 radius_of_acceptance
    ---
    ---

    """
    
    move_goal = MoveGoal()

    move_goal.controller_name = contr_name
    move_goal.target_pose.position.x = target_x
    move_goal.target_pose.position.y = target_y
    move_goal.target_pose.position.z = target_z

    move_goal.radius_of_acceptance = radius_of_acceptance


    return move_goal


class TaskManager():

    def __init__(self):

        rospy.init_node('move_to_and_inspect_point_sm', anonymous=False)


        hsm = StateMachine(outcomes=['finished statemachine'])
        
        with hsm:

            StateMachine.add(   'GO_TO_POINT',
                        SimpleActionState(  'pid_global',
                                            MoveAction,
                                            makeMoveGoal("pid_global_plan", -3.0, 0, -0.5, radius_of_acceptance = 2.0)),
                                            transitions = { "succeeded": 'INSPECT_POINT',
                                                            "preempted": 'INSPECT_POINT', 
                                                            "aborted": 'INSPECT_POINT' })    
            StateMachine.add(   'INSPECT_POINT',
                         SimpleActionState( 'inspect_point',
                                            MoveAction,
                                            makeMoveGoal("inspect_point", -3.0, 0.0, -0.5, radius_of_acceptance=2.0)),
                                            transitions = { 'succeeded': 'INSPECT_POINT',
                                                            "preempted": 'INSPECT_POINT', 
                                                            "aborted": 'INSPECT_POINT' })  
            
        

        


        intro_server = IntrospectionServer(str(rospy.get_name()), hsm,'/SM_ROOT')
        intro_server.start()
        hsm.execute()
        #patrol.execute()
        print("State machine execute finished")
        intro_server.stop()

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)




if __name__ == '__main__':
    try:
        TaskManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pathplanning state machine has been finished")