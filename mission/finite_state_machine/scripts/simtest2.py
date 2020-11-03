#!/usr/bin/env python

import rospy
from smach import Sequence
import smach
from helper import dp_move, los_move
from smach_ros import IntrospectionServer, SimpleActionState
from geometry_msgs.msg import Point, Pose, Quaternion #trenger ikke point?
from vortex_msgs.msg import MoveAction, MoveGoal #trenger ikke movegoal?
import time
from tf.transformations import quaternion_from_euler



class GateSearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],output_keys=['gate_search_output_cntrl_name','gate_search_output_target_pose'])
        
        #24.10 self.goal_position = None
        #self.goal = MoveGoal() #2410
        self.controller_name_LOS = 'LOS' #2410
        self.target_pose = Pose(None,Quaternion(*quaternion_from_euler(0, 0, 0))) #2410
        #subscribe her i stedet
    
    

    def execute(self, userdata):
        rospy.loginfo('executing GateSearchState')
        sub = rospy.Subscriber("goal_position", Point, self.callback)
        rospy.loginfo('waiting for goal position message')
        rospy.wait_for_message('goal_position', Point)
        #2410 rospy.loginfo('goal position message received %f,%f,%f', self.goal_position.x,self.goal_position.y,self.goal_position.z)
        sub.unregister()
        #24.10 userdata.gate_search_output = self.goal_position
        userdata.gate_search_output_cntrl_name = self.controller_name_LOS #2410
        userdata.gate_search_output_target_pose = self.target_pose #2410
        
        rospy.loginfo('sleeping')
        time.sleep(10) #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        return 'succeeded'
    
    def callback(self, goal_position):
        rospy.loginfo(rospy.get_caller_id() + "I heard %f,%f,%f", goal_position.x,goal_position.y,goal_position.z)
        #24.10 self.goal_position = goal_position
        self.target_pose.position = goal_position #2410

class FooState(smach.State):
    def __init__(self):
         smach.State.__init__(self, outcomes=['preempted', 'succeeded', 'aborted'],input_keys=['foo_state_input','foo_ctrl_name'])

    def execute(self,userdata):
        rospy.loginfo("executing foo state")
        pose = userdata.foo_state_input
        ctrl_name = userdata.foo_ctrl_name
        #2410 rospy.loginfo("foo class received %f,%f,%f", pos.x,pos.y,pos.z)
        rospy.loginfo("foo class received %f,%f,%f", pose.position.x,pose.position.y,pose.position.z)
        rospy.loginfo("foo class received " + ctrl_name)
        return 'succeeded'

def main():
    rospy.init_node('sim_sm2')

    simtest_sm = Sequence(outcomes=['preempted', 'succeeded', 'aborted'], connector_outcome='succeeded')

    #2410 simtest_sm.userdata.goal_position = None
    
    #simtest_sm.userdata.target_pose = None #2410
    #simtest_sm.userdata.controller_name_LOS = None #2410
    
    

    with simtest_sm:
        Sequence.add('REACH_DEPTH',dp_move(0,0),transitions={'succeeded':'GATE_SM'})

        gate_sm = smach.Sequence(outcomes=['preempted', 'succeeded', 'aborted'], connector_outcome='succeeded')

        gate_sm.userdata.target_pose_userdata = Pose(None,None) #2410
        gate_sm.userdata.controller_name_LOS = None #2410

        with gate_sm:

            #2410 Sequence.add('GATE_SEARCH',GateSearchState(), remapping={'gate_search_output':'goal_position'})
            Sequence.add('GATE_SEARCH',GateSearchState(), remapping={'gate_search_output_cntrl_name':'controller_name_LOS','gate_seatch_output_target_pose':'target_pose_userdata'}) #2410
            #Sequence.add('LOS_MOVE_TO_GATE',SimpleActionState('move',MoveAction,goal_slots=['controller_name','position'], remapping={'controller_name':'userdata_controller_name','position':'goal_position'}))
            Sequence.add('LOS_MOVE_TO_GATE', SimpleActionState('move',MoveAction,
                                                goal_slots=['controller_name','target_pose']),                      
                                                remapping={'controller_name':'controller_name_LOS','target_pose':'target_pose_userdata'})

            #2410 Sequence.add('FOO_STATE',FooState(), remapping={'foo_state_input':'goal_position'})
            Sequence.add('FOO_STATE',FooState(), remapping={'foo_state_input':'target_pose_userdata','foo_ctrl_name':'controller_name_LOS'})

        Sequence.add('GATE_SM',gate_sm)

    
    
    intro_server = IntrospectionServer(str(rospy.get_name()), simtest_sm,'/SM_ROOT')
    intro_server.start()


    try:
        simtest_sm.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("Pooltest failed: %s" % e)




if __name__ == '__main__':
    main()