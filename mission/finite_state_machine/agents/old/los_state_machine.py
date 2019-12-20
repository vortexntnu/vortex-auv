#!/usr/bin/env	python
import	rospy
from    time import sleep
from	smach	import	State, StateMachine		
from    nav_msgs.msg import Odometry    
from	smach_ros	 import	SimpleActionState, IntrospectionServer	
from    move_base_msgs.msg  import  MoveBaseAction, MoveBaseGoal
from    vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal

# Imported help functions from src/finite_state_machine
from    finite_state_machine import ControllerMode, WaypointClient, PathFollowingClient

#ENUM
OPEN_LOOP           = 0
POSE_HOLD           = 1
HEADING_HOLD        = 2
DEPTH_HEADING_HOLD  = 3 
DEPTH_HOLD          = 4
STAY_LEVEL          = 5
CONTROL_MODE_END    = 6


class Mission():

    def __init__(self):
        
        # ros init
        rospy.init_node('mission_fsm', anonymous=True)

        # rate
        self.rate = rospy.Rate(100) #Hz
        

if __name__ == '__main__':

    try:
        mission = Mission()
        los     = PathFollowingClient()

    except rospy.ROSInterruptException:
        rospy.loginfo("Unable to run constructor")

    # Define transit state
    transit = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    _goal = los.path_client(-5, 5, 10, -2, 0.2, 1.0)

    # State machine 
    with transit:
        StateMachine.add('transit', SimpleActionState('los_path',LosPathFollowingAction, goal=_goal), transitions={'succeeded':'succeeded'})

    
    sis = IntrospectionServer(str(rospy.get_name()), transit, '/SM_ROOT' + str(rospy.get_name()))
    sis.start()

    transit.execute()
    rospy.spin()
    sis.stop()
