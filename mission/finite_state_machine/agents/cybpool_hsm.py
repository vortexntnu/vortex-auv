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

class Navigation():

	def __init__(self):

		# my current pose
		self.vehicle_pose = Pose()
		self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)

	def positionCallback(self, msg):

		self.vehicle_pose = msg.pose.pose



# A list of tasks to be done
task_list = {'docking':['transit'],
			 'gate':['searching','detect','camera_centering','path_planning','tracking', 'passed'],
			 'pole':['searching','detect','camera_centering','path_planning','tracking', 'passed']
			}

def update_task_list(target, task):
    task_list[target].remove(task)
    if len(task_list[target]) == 0:
        del task_list[target]

class SearchForTarget(State):

	def __init__(self, target):
		State.__init__(self,outcomes=['found','unseen'],
							output_keys=['px_output'])

		self.target = target

		# initialize controller
		self.CameraPID = CameraPID()
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)

		# search for object
		if target == 'gate':
			self.sub_object = rospy.Subscriber('/gate_midpoint', CameraObjectInfo, self.objectDetectionCallback, queue_size=1)
		else:
			self.sub_object = rospy.Subscriber('/pole_midpoint', CameraObjectInfo, self.objectDetectionCallback, queue_size=1)

	def objectDetectionCallback(self, msg):

		""" detection frame
		(0,0)	increase->
		----------------> X
		|
		|
		| increase 
		|	 |
		|    v
		v
		
		Y

		"""

		self.object_px = msg.pos_x
		self.object_py = msg.pos_y
		self.object_fx = msg.frame_width
		self.object_fy = msg.frame_height
		self.object_confidence = msg.confidence
		self.object_distance = msg.distance_to_pole
		self.time = msg.header.stamp.to_sec()

	def alignWithTarget(self):

		self.CameraPID.swayController(0.0, self.object_px, self.time)

	def execute(self, userdata):

		rospy.loginfo('Searching for ' + self.target)
		sleep(0.5)

		if self.object_px >= 0.0 and self.object_py >= 0.0:
			rospy.loginfo(self.target + ' found')

			# output the object pixel position
			userdata.px_output = self.object_px
			return 'found'
		else:
			rospy.loginfo(self.target + ' not found')
			return 'unseen'

class TrackTarget(State):

	def __init__(self, target):
		State.__init__(self, outcomes=['succeeded','aborted','preempted'],
							input_keys=['px_input'])
		self.target = target

	def execute(self, userdata):

		rospy.loginfo('Pixel info: ' + str(userdata.px_input))

		return 'succeeded'


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
			nav_goal.desired_depth.z = self.search_depth
			nav_goal.sphereOfAcceptance = self.search_area_size
			los_path_state = SimpleActionState('los_path', LosPathFollowingAction,
												goal=nav_goal, 
												result_cb=self.nav_result_cb,
												exec_timeout=self.nav_timeout,
												server_wait_timeout=rospy.Duration(10.0))

			nav_transit_states[target] = los_path_state

		""" Create individual state machines for assigning tasks to each target zone """

		# Create a state machine for the orienting towards the gate subtask(s)
		sm_gate_tasks = StateMachine(outcomes=['found','unseen','aborted','succeeded','aborted','preempted'])

		# Then add the subtask(s)
		with sm_gate_tasks:
			# if gate is found, pass pixel info onto TrackTarget. If gate is not found, look again
			StateMachine.add('GATE_SEARCH', SearchForTarget('gate'), transitions={'found':'GATE_TRACK','unseen':'GATE_SEARCH'},
																	 remapping={'px_output':'object_px'})

			StateMachine.add('GATE_TRACK', TrackTarget('gate'), transitions={'succeeded':'', 'aborted':'', 'preempted':''},
																remapping={'px_input':'object_px'})


		""" Assemble a Hierarchical State Machine """

		# Initialize the HSM
		hsm_pool_patrol = StateMachine(outcomes=['succeeded','aborted','preempted'])

		# Build the HSM from nav states and target states

		with hsm_pool_patrol:

			""" Navigate to GATE in TERMINAL mode """
			StateMachine.add('TRANSIT_TO_GATE', nav_transit_states['gate'], transitions={'succeeded':'GATE_AREA','aborted':'RETURN_TO_DOCK','preempted':'RETURN_TO_DOCK'})
			StateMachine.add('GATE_AREA', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'GATE_AREA_STATIONKEEP','aborted':'RETURN_TO_DOCK','preempted':'RETURN_TO_DOCK'})
			StateMachine.add('GATE_AREA_STATIONKEEP', nav_terminal_states['gate'], transitions={'succeeded':'EXECUTE_GATE_TASKS','aborted':'RETURN_TO_DOCK','preempted':'RETURN_TO_DOCK'})

			""" When in GATE ZONE """		
			StateMachine.add('EXECUTE_GATE_TASKS', sm_gate_tasks, transitions={'found':'GATE_PASSED','unseen':'EXECUTE_GATE_TASKS','aborted':'RETURN_TO_DOCK'})		
			
			""" Transiting to gate """
			StateMachine.add('GATE_PASSED', ControlMode(OPEN_LOOP), transitions={'succeeded':'TRANSIT_TO_POLE','aborted':'RETURN_TO_DOCK','preempted':'GATE_AREA_STATIONKEEP'})
			StateMachine.add('TRANSIT_TO_POLE', nav_transit_states['pole'], transitions={'succeeded':'RETURN_TO_DOCK','aborted':'RETURN_TO_DOCK','preempted':'RETURN_TO_DOCK'})

			""" When aborted, return to docking """
			StateMachine.add('RETURN_TO_DOCK', ControlMode(POSE_HEADING_HOLD), transitions={'succeeded':'DOCKING','aborted':'','preempted':''})
			StateMachine.add('DOCKING', nav_terminal_states['docking'], transitions={'succeeded':'','aborted':'','preempted':''})

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