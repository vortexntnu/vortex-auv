#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from buoy import BuoySearch, BuoyConverge, BuoyExecute
from reach_depth import ReachDepth


def main():
    rospy.init_node('buoy_fsm_node')

    rospy.wait_for_service('send_positions')   #consider moving into individual state functions
          
    buoy_state_machine = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

    with buoy_state_machine:

        StateMachine.add('BUOY_PREPARE',
                        ReachDepth(),
                        transitions={'succeeded':'BUOY_SM'})

        buoy_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        with buoy_sm:
            StateMachine.add('BUOY_SEARCH',
                            BuoySearch(), 
                            transitions={'succeeded':'BUOY_CONVERGE'})
            
            
            StateMachine.add('BUOY_CONVERGE',
                            BuoyConverge(),
                            transitions={'succeeded' : 'BUOY_EXECUTE','aborted' : 'BUOY_SEARCH'}, 
                            remapping={'buoy_converge_output':'buoy'})
            
            StateMachine.add('BUOY_EXECUTE',
                            BuoyExecute())
        
        StateMachine.add('BUOY_SM',buoy_sm)

    intro_server = IntrospectionServer(str(rospy.get_name()), buoy_state_machine,'/SM_ROOT')    
    intro_server.start()

    try:
        buoy_state_machine.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("Prequalification test failed: %s" % e)       

if __name__ == '__main__':
    main() 