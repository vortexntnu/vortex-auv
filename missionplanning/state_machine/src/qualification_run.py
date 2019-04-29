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

class AC_handler():
    def __init__(self):
        self.depth_hold_ac = self.action_client('depth_hold_action_server', DepthHoldAction)

    def action_client(self, name, message_type):
        client = actionlib.SimpleActionClient(name, message_type)
        client.wait_for_server()
        #goal = DepthHoldGoal(depth = 2)
        #client.send_goal(goal)
        #ready = client.get_state()
        #client.wait_for_result()
        return client

    def cancel_all_goals(self):
        self.depth_hold_ac.cancel_all_goals()

def mission_trigger_callback(trigger_signal):
    print('Received signal')
    global mission_in_progress
    if(trigger_signal.data == True):
        mission_in_progress = not mission_in_progress
        print(mission_in_progress)

def request_preempt():
    global mission_in_progress
    if mission_in_progress:
        return False
    else:
        return True

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['doing','waiting'])
        # subscribe to signal
        print('Init')
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        #print('Executing')
        if mission_in_progress == False:
            #print('Waiting')
            self.rate.sleep()
            return 'waiting'
        else:
            print('Jumping')
            return 'doing'

class Cancel(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['canceld'])
        self.ac_handler = ac_handler

    def execute(self, userdata):
        #Kill all nodes
        self.ac_handler.cancel_all_goals()
        return 'canceld'


class Dive(smach.State):
    def __init__(self, ac_handler):
        smach.State.__init__(self, outcomes=['submerged','going','preempted'])
        print('Diving')
        self.ac_handler = ac_handler
        self.rate = rospy.Rate(10)


    def execute(self, userdata):
        print('diving')
        # turn on diving stuff and do that shit
        if request_preempt():
            return 'preempted'


        if self.ac_handler.depth_hold_ac.get_state() != 1:
            goal = DepthHoldGoal(depth = 2)
            self.ac_handler.depth_hold_ac.send_goal(goal)
            while(self.ac_handler.depth_hold_ac.get_state()!=1):
                self.rate.sleep()


        if False:#self.client.get_state[1]:
            self.rate.sleep()
            return 'submerged'
        else:
            self.rate.sleep()
            return 'going'





class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found','preempted'])
        #initialize stuff here

    def execute(self):
        if request_preempt():
            return 'preempted'
        #Do findGateStuff here
        if gate_found:
            return 'found'

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move_finised','gate_lost','preempted'])


    def execute(self):
        if request_preempt():
            return 'preempted'
        #Do driving motion here
        if gate_passed:
            return 'move_finished'
        elif gate_lost:
            return 'gate_lost'


def main():
    #rospy.init_node('action_client_py')
    rospy.init_node('Qualification_run')

    global mission_in_progress
    mission_in_progress = False

    rospy.Subscriber("mission_trigger", Bool, mission_trigger_callback)

    ac_handler = AC_handler()

    sm = smach.StateMachine(outcomes = ['Done'])

    sis = smach_ros.IntrospectionServer('Qualification_run_server', sm, '/SM_ROOT')
    sis.start()


    with sm:
        smach.StateMachine.add('Idle', Idle(),
                                transitions={'doing':'Dive', 'waiting':'Idle'})
        smach.StateMachine.add('Cancel', Cancel(ac_handler),
                                transitions={'canceld':'Idle'})
        smach.StateMachine.add('Dive', Dive(ac_handler),
                                transitions={'submerged':'Search', 'going':'Dive','preempted':'Cancel'})
        smach.StateMachine.add('Search', Search(),
                                transitions={'found':'Move', 'preempted':'Cancel'})
        smach.StateMachine.add('Move', Move(),
                                transitions={'move_finished':'Done', 'gate_lost':'Search','preempted':'Cancel'})


    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
