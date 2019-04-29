#!/usr/bin/env python
from __future__ import print_function
import rospy
import smach
import smach_ros
import smach_viewer
import roslaunch
import actionlib
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy
import actionlib_tutorials.msg
from std_msgs.msg import Bool

import actionlib
from depth_hold_action_server.msg import DepthHoldAction, DepthHoldGoal

def action_client():
    client = actionlib.SimpleActionClient('depth_hold_action_server', DepthHoldAction)
    client.wait_for_server()
    goal = DepthHoldGoal(depth = 2)
    client.send_goal(goal)
    ready = client.get_state()
    #client.wait_for_result()
    return client


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['doing','waiting'])
        # subscribe to signal 
        rospy.Subscriber("mission_start", Bool, self.callback)

    def callback(self, data):
        self.waitsignal = data

    def execute(self):
        if self.waitsignal == False:
            return 'waiting'

        else:
            return 'doing'


class Dive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['submerged','going','Stop'])
        self.client = action_client()

    def checksignal(self):
        if waitsignal == True:
            return 'Stop'

    def execute(self):
        print('diving')
        # turn on diving stuff and do that shit
        checksignal() #check if the ship has recieved a stop signal
        if self.client.get_state[1]:
            return 'submerged'
        else:
            return 'going'


class Cancel(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['canceld'])

    def execute(self):
        #Kill all nodes
        return 'cancled'


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['correct_heading'])
        result = action_client(order = 20)

    def execute(self):
        if gate_found:
            return 'found'


class Center(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Centered','Lost'])

        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,queue_size=1)
        self.sub_gate = rospy.Subscriber('camera_object_info',
                                          bool,queue_size=1000)
        result = action_client(order = 20)

    def execute(self):
        rospy.loginfo('Centering')
        if sub_gate == 1:
            return 'Centered'

def main():
    #rospy.init_node('action_client_py')
    rospy.init_node('Fibtest_test')
    sm = smach.StateMachine(outcomes = ['Done'])

    with sm:
        smach.StateMachine.add('Idle', Idle(),
                                transitions={'doing':'Dive', 'waiting':'Idle'})
        smach.StateMachine.add('Dive', Dive(),
                                transitions={'submerged':'Done', 'going':'Dive', 'Stop':'Cancel'})
        smach.StateMachine.add('Cancel', Cancel(),
                                transitions={'canceld':'Idle'})


    sis = smach_ros.IntrospectionServer('Fibtest_test_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    sis.stop()

if __name__ == '__main__':
    main()
