#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import	rospy
import  math
import numpy as np
from    time import sleep
from	collections import OrderedDict
from	smach	import	State, StateMachine		
from    nav_msgs.msg import Odometry    
from	smach_ros	 import	SimpleActionState, IntrospectionServer	
from    move_base_msgs.msg  import  MoveBaseAction, MoveBaseGoal
from    vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal
from 	vortex_msgs.msg import PropulsionCommand
from 	geometry_msgs.msg import Wrench, Pose
from 	tf.transformations import euler_from_quaternion

# import mission plan
from finite_state_machine.mission_plan import *

# import object detection
from	vortex_msgs.msg import CameraObjectInfo

# camera centering controller
from autopilot.autopilot import CameraPID

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

# A list of tasks to be done
task_list = {'Task1':['transit'],
			 'Task2':['transit'],
			 'Task3':['transit']
			}

def update_task_list(target, task):
    task_list[target].remove(task)
    if len(task_list[target]) == 0:
        del task_list[target]


class TaskManager():

	def __init__(self):

		# init node
		rospy.init_node('pool_patrol', anonymous=False)

		# Set the shutdown fuction (stop the robot)
		rospy.on_shutdown(self.shutdown)

		# Initilalize the mission parameters and variables
		setup_task_environment(self)

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



		for target in self.pool_locations.iterkeys():
			print(target)
			nav_goal = LosPathFollowingGoal()
			#nav_goal.prev_waypoint = navigation.vehicle_pose.position
			
			nav_goal.next_waypoint = self.pool_locations[target].position
			nav_goal.forward_speed.linear.x = self.transit_speed
			nav_goal.desired_depth.z = self.pool_locations[target].position.z
			nav_goal.sphereOfAcceptance = self.search_area_size
			los_path_state = SimpleActionState('los_path', LosPathFollowingAction,
												goal=nav_goal, 
												result_cb=self.nav_result_cb,
												exec_timeout=self.nav_timeout,
												server_wait_timeout=rospy.Duration(10.0))

			nav_transit_states[target] = los_path_state
		
		# Initialize the HSM
		hsm_pool_patrol = StateMachine(outcomes=['succeeded','aborted','preempted','passed','missed','unseen','found'])

		# Build the HSM from nav states and target states

		with hsm_pool_patrol:

			# Qualification Run
			StateMachine.add('GO_TO_DIVE', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'DIVE','aborted':'GO_TO_DIVE','preempted':'GO_TO_DIVE'})
			StateMachine.add('DIVE', nav_terminal_states['dive'], transitions={'succeeded':'OPEN_LOOP','aborted':'DIVE','preempted':'DIVE'})
			StateMachine.add('OPEN_LOOP', ControlMode(OPEN_LOOP), transitions={'succeeded':'GO_TO_GATE','aborted':'OPEN_LOOP','preempted':'OPEN_LOOP'})
			StateMachine.add('GO_TO_GATE', nav_transit_states['gate'], transitions={'succeeded':'ENABLE_POLE','aborted':'GO_TO_GATE','preempted':'GO_TO_GATE'})
			StateMachine.add('ENABLE_POLE', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'POLE1','aborted':'ENABLE_POLE','preempted':'ENABLE_POLE'})
			StateMachine.add('POLE1', nav_terminal_states['pole1'], transitions={'succeeded':'POLE2','aborted':'POLE1','preempted':'POLE1'})
			StateMachine.add('POLE2', nav_terminal_states['pole2'], transitions={'succeeded':'POLE3','aborted':'POLE2','preempted':'POLE2'})
			StateMachine.add('POLE3', nav_terminal_states['pole3'], transitions={'succeeded':'POLE4','aborted':'POLE3','preempted':'POLE3'})
			StateMachine.add('POLE4', nav_terminal_states['gate'], transitions={'succeeded':'OPEN_LOOP2','aborted':'POLE4','preempted':'POLE4'})
			StateMachine.add('OPEN_LOOP2', ControlMode(OPEN_LOOP), transitions={'succeeded':'GO_TO_START','aborted':'OPEN_LOOP2','preempted':'OPEN_LOOP2'})
			StateMachine.add('GO_TO_START', nav_transit_states['start'], transitions={'succeeded':'ENABLE_HOLD','aborted':'GO_TO_START','preempted':'GO_TO_START'})
			StateMachine.add('ENABLE_HOLD', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'HOLD','aborted':'ENABLE_HOLD','preempted':'ENABLE_HOLD'})
			StateMachine.add('HOLD', nav_terminal_states['start'], transitions={'succeeded':'OPEN_LOOP','aborted':'DIVE','preempted':'DIVE'})




			# Old Machine	
			#""" Navigate to corner1 """
			#StateMachine.add('los_corner1', nav_transit_states['corner1'], transitions={'succeeded':'control_mode_corner1','aborted':'los_corner1','preempted':'los_corner1'})
			#StateMachine.add('control_mode_corner1', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'dp_corner1','aborted':'control_mode_corner1','preempted':'control_mode_corner1'})
			#StateMachine.add('dp_corner1', nav_terminal_states['corner1'], transitions={'succeeded':'transition_corner1','aborted':'dp_corner1','preempted':'dp_corner1'})
			#StateMachine.add('transition_corner1', ControlMode(OPEN_LOOP), transitions={'succeeded':'los_corner2','aborted':'transition_corner1','preempted':'transition_corner1'})
			#""" Navigate to corner2 """
			#StateMachine.add('los_corner2', nav_transit_states['corner2'], transitions={'succeeded':'control_mode_corner2','aborted':'los_corner2','preempted':'los_corner2'})
			#StateMachine.add('control_mode_corner2', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'dp_corner2','aborted':'control_mode_corner2','preempted':'control_mode_corner2'})
			#StateMachine.add('dp_corner2', nav_terminal_states['corner2'], transitions={'succeeded':'transition_corner2','aborted':'dp_corner2','preempted':'dp_corner2'})
			#StateMachine.add('transition_corner2', ControlMode(OPEN_LOOP), transitions={'succeeded':'los_corner3','aborted':'transition_corner1','preempted':'transition_corner1'})
			#""" Navigate to corner3 """
			#StateMachine.add('los_corner3', nav_transit_states['corner3'], transitions={'succeeded':'control_mode_corner3','aborted':'los_corner3','preempted':'los_corner3'})
			#StateMachine.add('control_mode_corner3', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'dp_corner3','aborted':'control_mode_corner3','preempted':'control_mode_corner3'})
			#StateMachine.add('dp_corner3', nav_terminal_states['corner3'], transitions={'succeeded':'transition_corner3','aborted':'dp_corner3','preempted':'dp_corner3'})
			#StateMachine.add('transition_corner3', ControlMode(OPEN_LOOP), transitions={'succeeded':'los_corner4','aborted':'transition_corner1','preempted':'transition_corner1'})
			#""" Navigate to corner4 """
			#StateMachine.add('los_corner4', nav_transit_states['corner4'], transitions={'succeeded':'control_mode_corner4','aborted':'los_corner4','preempted':'los_corner4'})
			#StateMachine.add('control_mode_corner4', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'dp_corner4','aborted':'control_mode_corner4','preempted':'control_mode_corner4'})
			#StateMachine.add('dp_corner4', nav_terminal_states['corner4'], transitions={'succeeded':'transition_corner4','aborted':'dp_corner4','preempted':'dp_corner4'})
			#StateMachine.add('transition_corner4', ControlMode(OPEN_LOOP), transitions={'succeeded':'los_corner1','aborted':'transition_corner1','preempted':'transition_corner1'})
			



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