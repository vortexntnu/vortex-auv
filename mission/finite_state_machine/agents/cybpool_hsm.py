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


# Searching for Pole. When pole is found it passes to camera centering to center pole in frame. 
class SearchForTarget(State):
	def __init__(self,target):
		State.__init__(self,outcomes=['found','unseen','passed','missed'],
							output_keys=['px_output','fx_output', 'search_output', 'search_confidence_output'])
		self.target = target
		self.search_timeout = 30.0
		self.sampling_time = 0.2
		self.timer = 0.0
		self.task_status = 'missed'
		self.CameraPID = CameraPID()

		
		self.sub_object = rospy.Subscriber('/pole_midpoint', CameraObjectInfo, self.objectDetectionCallback, queue_size=1)
		self.sub_pose = rospy.Subscriber('/odometry/filtered',Odometry,self.positionCallback,queue_size=1)
		print(self.sub_pose)
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input',Wrench,queue_size=1)
		self.thrust_msg = Wrench()

	def objectDetectionCallback(self,msg):

		self.object_px = msg.pos_x
		self.object_py = msg.pos_y
		self.object_fx = msg.frame_height
		self.object_confidence = msg.confidence
		self.object_distance = msg.distance_to_pole

	def positionCallback(self, msg):

		self.vehicle_odom = msg
		self.time = msg.header.stamp.to_sec()

		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll,pitch,yaw) = euler_from_quaternion(orientation_list)

		self.psi = yaw
		


	def execute(self,userdata):
		
		
		#rospy.loginfo('Searching for ' + self.target)
		sleep(self.sampling_time)
		self.timer += self.sampling_time
		tau_heave = self.CameraPID.depthController(-0.5,self.vehicle_odom.pose.pose.position.z,self.time)
		self.thrust_msg.force.z = tau_heave

		if self.timer > self.search_timeout:
			return self.task_status

		#if self.object_px >= 0.0 and self.object_py >= 0.0:
		#	rospy.loginfo(self.target + ' found')
		#
		#	userdata.px_output = self.object_px
		#	userdata.fx_output = self.object_fx
		#	userdata.search_confidence_output = self.object_confidence
		#	userdata.search_output = 'found'
		#	self.task_status = 'passed'
		#	return 'found'

		else:
			#rospy.loginfo(self.target + ' not found')

			userdata.px_output = self.object_px
			userdata.fx_output = self.object_fx
			userdata.search_confidence_output = self.object_confidence
			userdata.search_output = 'unseen'
			tau_heading = self.CameraPID.headingController(self.psi + pi/8,self.psi,self.time)
			self.thrust_msg.torque.z = tau_heading	
			print(self.thrust_msg.torque.z)
			self.pub_thrust.publish(self.thrust_msg)
			return 'unseen'


		
		


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
			StateMachine.add('DIVE', nav_terminal_states['start'], transitions={'succeeded':'OPEN_LOOP','aborted':'DIVE','preempted':'DIVE'})
			StateMachine.add('OPEN_LOOP', ControlMode(OPEN_LOOP), transitions={'succeeded':'GO_TO_GATE','aborted':'OPEN_LOOP','preempted':'OPEN_LOOP'})
			StateMachine.add('GO_TO_GATE', nav_transit_states['gate'], transitions={'succeeded':'ENABLE_POLE','aborted':'GO_TO_GATE','preempted':'GO_TO_GATE'})
			StateMachine.add('ENABLE_POLE', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'SEARCH_FOR_POLE','aborted':'ENABLE_POLE','preempted':'ENABLE_POLE'})
			StateMachine.add('SEARCH_FOR_POLE',SearchForTarget('gate'),transitions={'found':'DIVE','unseen':'SEARCH_FOR_POLE','passed':'','missed':'OPEN_LOOP2'},															 remapping={'px_output':'object_px','fx_output':'object_fx','search_output':'object_search','search_confidence_output':'object_confidence'})
			StateMachine.add('OPEN_LOOP2', ControlMode(OPEN_LOOP), transitions={'succeeded':'GO_TO_START','aborted':'OPEN_LOOP2','preempted':'OPEN_LOOP2'})
			StateMachine.add('GO_TO_START', nav_transit_states['start'], transitions={'succeeded':'ENABLE_HOLD','aborted':'GO_TO_START','preempted':'GO_TO_START'})
			StateMachine.add('ENABLE_HOLD', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'HOLD','aborted':'ENABLE_HOLD','preempted':'ENABLE_HOLD'})
			StateMachine.add('HOLD', nav_terminal_states['start'], transitions={'succeeded':'OPEN_LOOP','aborted':'DIVE','preempted':'DIVE'})

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