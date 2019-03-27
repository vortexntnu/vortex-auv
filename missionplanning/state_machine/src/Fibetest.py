#!/usr/bin/env python
from __future__ import print_function
#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import smach_viewer
import roslaunch
import actionlib
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy
import actionlib_tutorials.msg

import actionlib
from depth_hold_action_server.msg import DepthHoldAction, DepthHoldGoal

def action_client():
    client = actionlib.SimpleActionClient('depth_hold_action_server', DepthHoldAction)
    client.wait_for_server()
    goal = DepthHoldGoal(depth = 2)
    client.send_goal(goal)

    #client.wait_for_result()
    return client

class Dive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['submerged','going'])
        self.client = action_client()
    def execute(self, userdata):
        print('diving')

        if self.client.get_state() == 3:
            print('Done')
            return 'submerged'
        else:
            return 'going'


class Heading(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['correct_heading'])
        result = action_client(order = 20)

    def execute(self, userdata):
        if heading == ref:
            return 'correct_heading'


class Center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Centered','Lost'])

        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',
                                          bool,queue_size=1000)
        result = action_client(order = 20)

    def execute(self, userdata):
        rospy.loginfo('Centering')
        if sub_gate == 1:
            return 'Centered'

def main():
    #rospy.init_node('action_client_py')
    rospy.init_node('Fibtest_test')
    sm = smach.StateMachine(outcomes = ['Done'])

    with sm:
        smach.StateMachine.add('Dive', Dive(),
                                transitions={'submerged':'Done', 'going':'Dive'})
        #smach.StateMachine.add('Heading', Heading(),
        #                        transitions={'correct_heading':'Center'})
        #smach.StateMachine.add('Center', Center(),
        #                        transitions={'Centered':'Done'})

    sis = smach_ros.IntrospectionServer('Fibtest_test_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    sis.stop()

if __name__ == '__main__':
    main()
