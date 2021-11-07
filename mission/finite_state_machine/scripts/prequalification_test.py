#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach
from fsm_helper import dp_move, los_move
from smach_ros import IntrospectionServer, SimpleActionState
from geometry_msgs.msg import Point
from vortex_msgs.msg import MoveAction, MoveGoal
from sm_classes import GateSearchState
from nav_msgs.msg import Odometry
import copy

def main():
    rospy.init_node('prequalification_test')
          
    prequalification_state_machine = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    
    move_action_server = '/guidance/move'

    odom = Odometry(None,None,None,None)
    def odom_cb(odom_msg):
        odom = odom_msg
    rospy.Subscriber("odometry/filtered", Odometry, odom_cb)                  
    
    with prequalification_state_machine:
                
        StateMachine.add('REACH_DEPTH',
                        dp_move(0,0),
                        transitions={'succeeded':'GATE_SM'})
            
        gate_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        gate_sm.userdata.goal_position = Point(None,None,None)        

        with gate_sm:
            
            StateMachine.add('GATE_SEARCH',
                            GateSearchState(), 
                            transitions={'succeeded':'LOS_MOVE_TO_GATE'}, 
                            remapping={'gate_search_output':'goal_position'}) #??      
                        
            def gate_goal_cb(userdata, goal): #What does this function do?? I think we need 'getMiddlePointGate' in this function
                gate_goal = MoveGoal()
                gate_goal.target_pose.position = userdata.goal_position                
                gate_goal.guidance_type = "LOS"
                return gate_goal
                        
            StateMachine.add('LOS_MOVE_TO_GATE',
                            SimpleActionState(move_action_server,
                                                MoveAction,
                                                goal_cb=gate_goal_cb,
                                                input_keys=['goal_position']),
                            transitions={'succeeded':'PREPARE_MOVE_THROUGH','aborted':'GATE_SEARCH'})
                            
            
            def prep_goal_cb(userdata, goal): #??
                prep_goal = MoveGoal()                
                prep_goal.target_pose.position = userdata.goal_position
                x_dist = odom.pose.pose.position.x - userdata.goal_position.x
                prep_goal.target_pose.position.x = userdata.goal_position.x + (x_dist/abs(x_dist))/4                 
                prep_goal.guidance_type = "PositionHold" 
                return prep_goal

            StateMachine.add('PREPARE_MOVE_THROUGH',
                            SimpleActionState(move_action_server,
                                            MoveAction,
                                            goal_cb=prep_goal_cb,
                                            input_keys=['goal_position']),
                            transitions={'succeeded':'LOS_MOVE_THROUGH_GATE'})                            
           
            def through_goal_cb(userdata,goal): #??
                through_goal = MoveGoal() #Must here move goal to pole-coordinates
                through_goal.target_pose.position = copy.deepcopy(userdata.goal_position)
                through_goal.target_pose.position.x += 1
                through_goal.guidance_type = 'LOS'
                return through_goal

            StateMachine.add('LOS_MOVE_THROUGH_GATE',
                            SimpleActionState(move_action_server,
                                            MoveAction,
                                            goal_cb=through_goal_cb,
                                            input_keys=['goal_position']))
                
        StateMachine.add('GATE_SM',gate_sm)


        pole_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        pole_sm.userdata.goal_position = Point(None,None,None)

        with pole_sm:

            StateMachine.add('POLE_SEARCH',
                            PoleSearchState(), #Must make PoleSearchState
                            transitions={'succeeded':'LOS_MOVE_TO_POLE'}) 
                            #remapping={'gate_search_output':'goal_position'}) Unsure what this does -copied from gate_sm
            

            StateMachine.add('LOS_MOVE_TO_POLE',
                            SimpleActionState(), #must make SimpleActionState
                            transitions={'succeeded':'PREPARE_MOVE_AROUND_POLE','aborted':'POLE_SEARCH'})

            StateMachine.add('PREPARE_MOVE_AROUND_POLE',
                            SimpleActionState(), #Must make SimpleActionState
                            transitions={'succeeded': 'MOVE_AROUND_POLE'})

            StateMachine.add('MOVE_AROUND_POLE', 
                            SimpleActionState())
                            

        StateMachine.add('POLE_SM', pole_sm)


    intro_server = IntrospectionServer(str(rospy.get_name()), prequalification_state_machine,'/SM_ROOT')    
    intro_server.start()

    try:
        prequalification_state_machine.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("PreqTest failed: %s" % e)


def getMiddlePointGate(upperLeft, bottomRight):
    "UpperLeft/bottomRight = [x,y,z]-coordinates, x,y,z must be float"
    x = (upperLeft[0] + bottomRight[0])/2
    y = (upperLeft[1] + bottomRight[1])/2
    z = (upperLeft[2] + bottomRight[2])/2
    return [x,y,z]

if __name__ == '__main__':
    main()
