#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach
from helper import dp_move, los_move
from smach_ros import IntrospectionServer, SimpleActionState
from geometry_msgs.msg import Point
from vortex_msgs.msg import MoveAction, MoveGoal
from sm_classes import GateSearchState


def main():
    rospy.init_node('simulator_state_machine')
          
    simulator_state_machine = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    
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
                gate_goal.controller_name = "LOS"
                return gate_goal
                        
            StateMachine.add('LOS_MOVE_TO_GATE',
                            SimpleActionState('move',
                                                MoveAction,
                                                goal_cb=gate_goal_cb,
                                                input_keys=['goal_position']),
                            transitions={'succeeded':'LOS_MOVE_THROUGH_GATE','aborted':'GATE_SEARCH'},
                            remapping={'goal_position':'goal_position'})                     
            
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