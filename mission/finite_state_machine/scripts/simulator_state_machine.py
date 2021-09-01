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


def main():
    rospy.init_node('simulator_state_machine')
    
    move_action_server = '/guidance/move'
          
    simulator_state_machine = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    
    
    odom = Odometry(None,None,None,None)
    def odom_cb(odom_msg):
        odom = odom_msg
    rospy.Subscriber("odometry/filtered", Odometry, odom_cb)                  
    
    with simulator_state_machine:
                
        StateMachine.add('REACH_DEPTH',
                        dp_move(0,0),
                        transitions={'succeeded':'GATE_SM'})
            
        gate_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        gate_sm.userdata.goal_position = Point(None,None,None)        

        with gate_sm:
            
            StateMachine.add('GATE_SEARCH',
                            GateSearchState(), 
                            transitions={'succeeded':'LOS_MOVE_TO_GATE'},
                            remapping={'gate_search_output':'goal_position'})         
                        
            def gate_goal_cb(userdata, goal):
                gate_goal = MoveGoal()
                gate_goal.target_pose.position = userdata.goal_position                
                gate_goal.guidance_type = "LOS"
                return gate_goal
                        
            StateMachine.add('LOS_MOVE_TO_GATE',
                            SimpleActionState(move_action_server,
                                                MoveAction,
                                                goal_cb=gate_goal_cb,
                                                input_keys=['goal_position']),
                            transitions={'succeeded':'PREPARE_MOVE_THROUGH','aborted':'GATE_SEARCH'},
                            remapping={'goal_position':'goal_position'})
            
            def prep_goal_cb(userdata, goal):
                prep_goal = MoveGoal()                
                prep_goal.target_pose.position = userdata.goal_position
                x_dist = odom.pose.pose.position.x - userdata.goal_position.x
                prep_goal.target_pose.position.x = userdata.goal_position.x + (x_dist/abs(x_dist))/2                 
                prep_goal.guidance_type = "PositionHold" 
                return prep_goal

            StateMachine.add('PREPARE_MOVE_THROUGH',
                            SimpleActionState(move_action_server,
                                                MoveAction,
                                                goal_cb=prep_goal_cb,
                                                input_keys=['goal_position']),
                            transitions={'succeeded':'LOS_MOVE_THROUGH_GATE'},
                            remapping={'goal_pos_input':'goal_position'})                     
            
            StateMachine.add('LOS_MOVE_THROUGH_GATE',
                            los_move(8,1.5),
                            transitions={'aborted':'GATE_SEARCH'})                  
                
        StateMachine.add('GATE_SM',gate_sm)

            
    intro_server = IntrospectionServer(str(rospy.get_name()), simulator_state_machine,'/SM_ROOT')    
    intro_server.start()

    try:
        simulator_state_machine.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("Pooltest failed: %s" % e)



if __name__ == '__main__':
    main()
