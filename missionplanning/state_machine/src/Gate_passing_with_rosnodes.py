#!/usr/bin/env python
import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import roslaunch
import actionlib
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy
import Vortex_start_node.msg

from __future__ import print_function
import actionlib

def action_client(message)
    client = actionlib.SimpleActionClient('action', Vortex_start_node.msg)
    client.wait_for_server()
    goal = actionlib_start_node.msg(message)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_resutl

class Dive(smach.State):
    def __init__(self):
        result = action_client('start_Dive')
        
    def execute(self):
        if depth == 1:
            return 'submerged'


class Heading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['correct_heading'])
        result = action_client('start_heading')

        def execute(self):
            if heading == ref:
                return 'correct_heading'


class Center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Centered','Lost'])
        
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',
                                          bool,queue_size=1000)
        result = action_client('start_center')

    def execute(self):
        rospy.loginfo('Centering')
        if sub_gate == 1:
            return 'Centered'
  #      if sub_gate == 0:   #if we loose the gate
   #         return 'Lost'
    #    elif sub_gate == 1 and gate_centered == 1:  #if we see the gate and it is in center
     #       return 'Centered'




'''

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes={'Found','Not_Found'},output_keys={'Found_gate'})
        self.motion_search = [0,0,0,0,0,0.2]

        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',
                                          bool,queue_size=1000)

    def execute(self):
        rospy.loginfo('Searching')
        while self.sub_gate == None:
            #Crappy search pattern for gate
            #Will be massivly improved
            self.pub_motion(self.motion_search)
        return 'Found', 'Found_gate'


class Drive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes={'Complete','Complete_bouy','Lost_boy', 'Lost_gate'}, input_keys={'Input_gate','Input_bouy'})

        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',
                                          bool,queue_size=1000)

    def execute(self):
        if 'Input_gate' == 1:
            while sub_gate == 1: #Gate is visable and centered
                roslaunch Drive()   #makes the drone move forward
                if gate_passed == 1: #The gate has been passed
                    rosnode kill -Camera_centering
                    return 'Complete'
                else: #Gate lost for some reason
                    return 'Lost'
        if 'Input_bouy' == 1:
            while sub_bouy == 1: #Gate is visable and centered
                roslaunch Drive()   #makes the drone move forward
                if bouy_passed == 1: #The gate has been passed
                    rosnode kill -Camera_centering
                    return 'Complete_bouy'
                else: #Gate lost for some reason
                    return 'Lost_bouy'


class Search_Bouy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes={'Found_b'},output_keys={'Found_bouy'})
        self.motion_search = [0,0,0,0,0,0.2]

        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_bouy = rospy.Subscriber('camera_object_info',      #Do we see the bouy?
                                          bool,queue_size=1000)

    def execute(self):
        rospy.loginfo('Searching_b')
        while self.sub_bouy == None:
            #Crappy search pattern for gate
            #Will be massivly improved
            self.pub_motion(self.motion_search)
        return 'Found_b', 'Found_bouy'

'''
def main():
    rospy.init_node('action_client_py')
    rospy.init_node("Mini_basseng_test")
    sm = smach.StateMachine(outcomes = {'Done'})

    sis = smach_ros.IntrospectionServer('Mini_basseng_test_server', sm, '/SM_ROOT')
    sis.start()

    with sm:
        smach.StateMachine.add('Dive', Dive(),
                                transitions={'submerged':'Heading'})
        smach.StateMachine.add('Heading', Heading(),
                                transitions={'correct_heading':'Center'})
        smach.StateMachine.add('Center', Center(),
                                transitions={'Centered':'Done'})
 #       smach.StateMachine.add('Search', Search(),
  #                              transitions={'Found':'Center'}
   #                             remapping={'Found_gate':'sm_data1'})
    #    smach.StateMachine.add('Center', Center(),
     #                           transitions={'Centered':'Drive'})
      #  smach-StateMachine.add('Search_b', Search_Bouy,
       #                         transitions = {'Found_b':'Center'}
        #                        remapping={'Found_bouy','sm_data2'})
        #smach-StateMachine.add('Drive', Drive(),
          #                      transitions={'Lost':'Search',
           #                                 'Complete','Search_b'
            #                                'Lost_bouy':'Seach_b',
             #                               'Complete_bouy', 'Search'}
              #                  remapping={'Input_bouy':'sm_data2',
               #                             '_gate':'sm_data1'})


    outcome = sm.execute()

if __name__ == '__main__':
    main()
