#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

from gate_choose_side import GateSearch, GateConverge, GateExecute
from pole import PoleSearch, PoleConverge, PoleExecute
from reach_depth import ReachDepth

def main():
    rospy.init_node('robosub_fsm')

    rospy.wait_for_service('send_positions')   #consider moving into individual state functions
          
    robosub_state_machine = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
                
    with robosub_state_machine:
        ##COIN FLIP
        #StateMachine.add('ROBOSUB_PREPARE',
        #                ReachDepth(),
        #                transitions={'succeeded':'GATE_SM'})

        ##GATE
        gate_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
        with gate_sm:
            StateMachine.add('GATE_SEARCH',
                        GateSearch(), 
                        transitions={'succeeded':'GATE_CONVERGE'},
                        remapping={'gate_search_output':'gate'})

            StateMachine.add('GATE_CONVERGE',
                        GateConverge(),
                        transitions={'succeeded' : 'GATE_EXECUTE','aborted' : 'GATE_SEARCH'}, 
                        remapping={'gate_converge_output':'gate'})
            
            StateMachine.add('GATE_EXECUTE',
                        GateExecute())
        
        StateMachine.add('GATE_SM',gate_sm )
        ##PATH

        ##BUOYS

        ##TORPEDO

        ##Octagon/Bins - undecided whether these will be performed for 2022

        ##Resurface


    # intro_server = IntrospectionServer(str(rospy.get_name()), robosub_state_machine,'/SM_ROOT')    
    # intro_server.start()

    try:
        robosub_state_machine.execute()
        # intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == '__main__':
    main()