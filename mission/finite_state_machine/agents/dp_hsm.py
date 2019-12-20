#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import	rospy
from    time import sleep
from	collections import OrderedDict
from	smach	import	State, StateMachine		
from    nav_msgs.msg import Odometry    
from	smach_ros	 import	SimpleActionState, IntrospectionServer	
from    move_base_msgs.msg  import  MoveBaseAction, MoveBaseGoal
from    vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal
from 	vortex_msgs.msg import PropulsionCommand
from 	geometry_msgs.msg import Wrench, Pose

# import mission plan
from finite_state_machine.mission_plan import *


# Imported help functions from src/finite_state_machine/
from    finite_state_machine import ControllerMode, WaypointClient, PathFollowingClient

#ENUM
OPEN_LOOP           = 0
POSE_HOLD           = 1
HEADING_HOLD        = 2
DEPTH_HEADING_HOLD  = 3 
DEPTH_HOLD          = 4
POSE_HEADING_HOLD   = 5
CONTROL_MODE_END    = 6

class ControlMode(State):

    def __init__(self, mode):
        State.__init__(self, ['succeeded','aborted','preempted'])
        self.mode = mode
        self.control_mode = ControllerMode()

    def execute(self, userdata):

        # change control mode
        self.control_mode.change_control_mode_client(self.mode)
        rospy.loginfo('changed DP control mode to: ' + str(self.mode) + '!')
        return 'succeeded'

class Navigation():

	def __init__(self):

		# my current pose
		self.vehicle_pose = Pose()

		# subscriber
		self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)

	def positionCallback(self, msg):

		self.vehicle_pose = msg.pose.pose

# A list of tasks to be done
task_list = {'startup':['transit'],
			 'gate':['orient_to_gate','detect_gate','centering_gate','pass_gate'],
			 'pole':['orient_to_pole','detect_pole','centering_pole','plan_path','execute_path'],
			 'hexagon':['surface']
			}

class OrientToTarget(State):

	def __init__(self, target, timer):
		State.__init__(self,outcomes=['succeeded','aborted','preempted'])

		self.task = 'orient_to_target'
		self.target = target
		self.timer = timer
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)

	def execute(self, userdata):
		rospy.loginfo('orienting towards' + str(self.target))

		sleep(5)

		rospy.loginfo('Done orienting towards ' + str(self.target) + '!')

		return 'succeeded'


def update_task_list(target, task):
    task_list[target].remove(task)
    if len(task_list[target]) == 0:
        del task_list[garget]


class TaskManager():

	def __init__(self):

		# init node
		rospy.init_node('pool_patrol', anonymous=False)

		# Set the shutdown fuction (stop the robot)
		rospy.on_shutdown(self.shutdown)

		# Initilalize the mission parameters and variables
		setup_task_environment(self)

		# get vehicle pose
		navigation = Navigation()

		# Turn the target locations into SMACH MoveBase and LosPathFollowing action states
		nav_terminal_states = {}
		nav_transit_states = {}

		# DP controller
		for target in self.pool_locations.iterkeys():
			nav_goal = MoveBaseGoal()
			nav_goal.target_pose.header.frame_id = 'odom'
			nav_goal.target_pose.pose = self.pool_locations[target]
			move_base_state = SimpleActionState('move_base', MoveBaseAction,
												goal=nav_goal, 
												result_cb=self.nav_result_cb,
												exec_timeout=self.nav_timeout,
												server_wait_timeout=rospy.Duration(10.0))

			nav_terminal_states[target] = move_base_state

		# Path following
		for target in self.pool_locations.iterkeys():
			nav_goal = LosPathFollowingGoal()
			#nav_goal.prev_waypoint = navigation.vehicle_pose.position
			nav_goal.next_waypoint = self.pool_locations[target].position
			nav_goal.forward_speed.linear.x = 0.2
			nav_goal.desired_depth.z = -0.5
			nav_goal.sphereOfAcceptance = 0.5
			los_path_state = SimpleActionState('los_path', LosPathFollowingAction,
												goal=nav_goal, 
												result_cb=self.nav_result_cb,
												exec_timeout=self.nav_timeout,
												server_wait_timeout=rospy.Duration(10.0))

			nav_transit_states[target] = los_path_state

		""" Create individual state machines for assigning tasks to each target zone """

		# Create a state machine for the orienting towards the gate subtask(s)
		sm_gate_task = StateMachine(outcomes=['succeeded','aborted','preempted'])

		# Then add the subtask(s)
		with sm_gate_task:
			StateMachine.add('ORIENT_TO_GATE', OrientToTarget('gate', 5), transitions={'succeeded':'','aborted':'','preempted':''})


		""" Assemble a Hierarchical State Machine """

		# Initialize the HSM
		hsm_pool_patrol = StateMachine(outcomes=['succeeded','aborted','preempted'])

		# Build the HSM from nav states and target states

		with hsm_pool_patrol:

			""" Navigate to GATE in TERMINAL mode """
			StateMachine.add('DOCKING', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'p0','aborted':'','preempted':''})
			StateMachine.add('p0', nav_terminal_states['p0'], transitions={'succeeded':'p1','aborted':'','preempted':''})
			StateMachine.add('p1', nav_terminal_states['p1'], transitions={'succeeded':'p2','aborted':'','preempted':''})
			StateMachine.add('p2', nav_terminal_states['p2'], transitions={'succeeded':'p3','aborted':'','preempted':''})
			StateMachine.add('p3', nav_terminal_states['p3'], transitions={'succeeded':'p4','aborted':'','preempted':''})
			StateMachine.add('p4', nav_terminal_states['p4'], transitions={'succeeded':'p5','aborted':'','preempted':''})
			StateMachine.add('p5', nav_terminal_states['p5'], transitions={'succeeded':'p6','aborted':'','preempted':''})
			StateMachine.add('p6', nav_terminal_states['p6'], transitions={'succeeded':'','aborted':'','preempted':''})

		# Create and start the SMACH Introspection server

		intro_server = IntrospectionServer(str(rospy.get_name()),hsm_pool_patrol,'/SM_ROOT')
		intro_server.start()

		# Execute the state machine
		hsm_outcome = hsm_pool_patrol.execute()
		intro_server.stop()

	def nav_result_cb(self, userdata, status, result):

		if status == GoalStatus.PREEMPTED:
			rospy.loginfo("Waypoint preempted")
		if status == GoalStatus.SUCCEEDED:
			rospy.loginfo("Waypoint succeeded")

	def shutdown(self):
		rospy.loginfo("stopping the AUV...")
		#sm_nav.request_preempt()
		rospy.sleep(10)


if __name__ == '__main__':

	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Mission pool patrol has been finished")