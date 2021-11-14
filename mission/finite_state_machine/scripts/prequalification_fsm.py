#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach
from fsm_helper import dp_move, los_move
from smach_ros import IntrospectionServer, SimpleActionState
from geometry_msgs.msg import Point
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sm_classes import GateSearchState
from nav_msgs.msg import Odometry
import copy

def main():
    rospy.init_node('prequalification_test')
          
    prequalification_state_machine = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    
   guidance_interface_dp_action_server="guidance_interface/dp_server"
   guidance_interface_los_action_server="guidance_interface/los_server"
   #virtual target action server ...

    odom = Odometry(None,None,None,None)
    def odom_cb(odom_msg):
        odom = odom_msg
    rospy.Subscriber("odometry/filtered", Odometry, odom_cb)                  
    
    with prequalification_state_machine:
                
        StateMachine.add('REACH_DEPTH',
                        dp_move(0,0), #moves to depth 0.5 (hard coded in dp_move, see fsm helper)
                        transitions={'succeeded':'GATE_SM'}) #is this ok?
            
        gate_sm = StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        gate_sm.userdata.goal_position = Point(None,None,None)        

        with gate_sm:
            
            StateMachine.add('GATE_SEARCH',
                            GateSearchState(), 
                            transitions={'succeeded':'LOS_MOVE_TO_GATE'}, 
                            remapping={'gate_search_output':'goal_position'})      
                        
            def gate_goal_cb(userdata, goal):  #This is where we take in the position(s) from landmarks and generate the new waypoint for LOS
                gate_goal = LosPathFollowingGoal()
                # possibly some function 'getMiddlePointGate' here
                #gate_goal.target_pose.position = userdata.goal_position    
                gate_goal.forward_speed = rospy.get_param('~transit_speed',0.3)  
                gate_goal.sphereOfAcceptance = rospy.get_param('~sphere_of_acceptance',0.5)      
                gate_goal.desired_depth = 0.5    
                return gate_goal
                        
            StateMachine.add('LOS_MOVE_TO_GATE', #potentially replace with VTPF (virtual target path following)
                            SimpleActionState(guidance_interface_los_action_server,
                                                LosPathFollowingAction,
                                                goal_cb=gate_goal_cb,
                                                input_keys=['goal_position']),
                            transitions={'succeeded':'PREPARE_MOVE_THROUGH','aborted':'GATE_SEARCH'})
                            
            
            def prep_goal_cb(userdata, goal):
                prep_goal = MoveBaseGoal()                
                prep_goal.target_pose.pose.position = userdata.goal_position #(or whichever point from userfdata.goal_position we care about)
                x_dist = odom.pose.pose.position.x - userdata.goal_position.x
                prep_goal.target_pose.pose.position.x = userdata.goal_position.x + (x_dist/abs(x_dist))/4        #(probably dont need this)         
                return prep_goal

            StateMachine.add('PREPARE_MOVE_THROUGH',
                            SimpleActionState(guidance_interface_dp_action_server,
                                            MoveBaseAction,
                                            goal_cb=prep_goal_cb,
                                            input_keys=['goal_position']),
                            transitions={'succeeded':'LOS_MOVE_THROUGH_GATE'})                            
           
            def through_goal_cb(userdata,goal):
                through_goal = LosPathFollowingGoal()
                through_goal.target_pose.pose.position = copy.deepcopy(userdata.goal_position) #what is copy.deepcopy?
                through_goal.target_pose.pose.position.x += 1 #why?

                through_goal.forward_speed = rospy.get_param('~transit_speed',0.3)  
                through_goal.sphereOfAcceptance = rospy.get_param('~sphere_of_acceptance',0.5)      
                through_goal.desired_depth = 0.5    
                return through_goal

            StateMachine.add('LOS_MOVE_THROUGH_GATE',
                            SimpleActionState(guidance_interface_los_action_server,
                                            LosPathFollowingAction,
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