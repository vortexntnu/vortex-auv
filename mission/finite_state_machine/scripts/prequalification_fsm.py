#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach
from fsm_helper import dp_move, los_move
from smach_ros import IntrospectionServer, SimpleActionState
from geometry_msgs.msg import Point
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import sm_classes

from gate import GateSearch, GateConverge, GateExecute
from pole import PoleSearch, PoleConverge, PoleExecute

import copy

def main():
    rospy.init_node('prequalification_fsm')

    rospy.wait_for_service('send_positions')   #consider moving into individual state functions
          
    prequalification_state_machine = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
                
    with prequalification_state_machine:
                
        # StateMachine.add('REACH_DEPTH',
        #                 dp_move(0,0),
        #                 transitions={'succeeded':'GATE_SM'})
            
        gate_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        with gate_sm:

            StateMachine.add('GATE_SEARCH',
                            GateSearch(), 
                            transitions={'succeeded':'MOVE_TO_GATE'},
                            remapping={'gate_search_output':'gate_position'})
            
            
            StateMachine.add('MOVE_TO_GATE',
                            GateConverge())#,
                            #transitions={'succeeded' : 'MOVE_THROUGH_GATE','aborted' : 'GATE_SEARCH'})
            
            StateMachine.add('MOVE_THROUGH_GATE',
                            GateExecute())
        
                
        StateMachine.add('GATE_SM',gate_sm)#,
                        #transitions={'succeeded':'POLE_SM'} )


        pole_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        with pole_sm:

            StateMachine.add('POLE_SEARCH',
                             PoleSearch(),
                             transitions={'succeeded':'MOVE_TO_POLE'}, 
                             remapping={'pole_search_output':'pole_position'}) 
        
            StateMachine.add('MOVE_TO_POLE',
                            PoleConverge(),
                            transitions={'succeeded':'MOVE_AROUND_POLE', 'aborted':'POLE_SEARCH'})   

            StateMachine.add('MOVE_AROUND_POLE',
                            PoleExecute())                    

        StateMachine.add('POLE_SM', pole_sm)


    intro_server = IntrospectionServer(str(rospy.get_name()), prequalification_state_machine,'/SM_ROOT')    
    intro_server.start()

    try:
        prequalification_state_machine.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("PreqTest failed: %s" % e)


if __name__ == '__main__':
    main()