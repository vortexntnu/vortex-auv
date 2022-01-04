#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Documented by Christopher Strom and Jae Hyeong Hwang
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
import numpy as np
import math
from vortex_msgs.msg import GuidanceData
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# dynamic reconfigure
from dynamic_reconfigure.server import Server
from los_guidance.cfg import LOSConfig

# vtf
from guidance_and_control_node import VtfGuidanceAndControlNode, create_wrenchstamped_msg

# action message
import actionlib
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal, VtfPathFollowingResult


class VtfPathFollowing(object):
	"""
	This is the wrapper class for the VtfGuidanceAndControl class. 

	Attributes:
		_result		A vortex_msgs action, true if a goal is set within the
					sphereof acceptance, false if not
	
	Nodes created:
		vtf_server

	Subscribes to:
		/odometry/filtered
	
	Publishes to:
		/guidance/los_data
		/auv/los_desired
	"""

	# create messages that are used to send feedback/result
	_result = VtfPathFollowingResult()

	def __init__(self):
		"""
		To initialize the ROS wrapper, the node, subscribers
		 are set up, as well as the action server.
		"""

		rospy.init_node('vtf_server')
		while rospy.get_time() == 0:
			continue

		'''Flag to indicate if a vtf guidance is active or not'''
		self.publish_guidance_data = False

		# parameters
		rate = rospy.get_param("/guidance/vtf/rate", default=20)
		self.ros_rate = rospy.Rate(rate)

		# constructor object
		self.vtf = VtfGuidanceAndControlNode()

		# Action server, see https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/simple_action_server.py
		self.action_server = actionlib.SimpleActionServer(name='vtf_action_server', ActionSpec=VtfPathFollowingAction, auto_start=False)
		self.action_server.register_goal_callback(self.goal_cb)
		self.action_server.start()

		rospy.loginfo("vtf guidance initiated")

	def spin(self):
		while not rospy.is_shutdown():
			try:
				if self.publish_guidance_data:
					self.vtf.publish_control_forces()
					self.statusActionGoal()
					self.ros_rate.sleep()
			except rospy.ROSInterruptException:
				pass

		
	def statusActionGoal(self):
		"""
		Checks if a preempt request has been set or if the goal was reached. 
		For both cases it publishes a 0 wrench to the thruster manager 
		and sets the action server to preempted or succeeded. 
		"""
		if self.action_server.is_preempt_requested():
			rospy.loginfo("Preempted requested by vtf path client")
			self.publish_guidance_data = False
			msg = create_wrenchstamped_msg([0,0,0,0,0,0], rospy.get_rostime())
			self.vtf.pub.publish(msg)
			self.action_server.set_preempted()

		# succeeded
		if self.vtf.goal_reached:
			self._result.terminalSector = True
			self.publish_guidance_data = False
			self.vtf.goal_reached = False
			self.action_server.set_succeeded(self._result, text="goal completed")
			msg = create_wrenchstamped_msg([0,0,0,0,0,0], rospy.get_rostime())
			self.vtf.pub.publish(msg)
			

	def goal_cb(self):
		"""
		The goal callback for the action server.

		Once a goal has been recieved from the client, self.publish_guidance_data is set to True
		This means that this node will start publishing data for the controller
		"""

		_goal = self.action_server.accept_new_goal()
		rospy.logdebug("vtf_guidance recieved new goal")

		#reset goal reached
		self.vtf.goal_reached = False
		# set goal, the first item will be replaced by current position by the vtf controller so init with dummy:
		self.vtf.waypoints = [[6,6,6]] 
		for wp in _goal.waypoints:
			self.vtf.waypoints.append([wp.y,wp.x,-wp.z])

		self.vtf.new_path_recieved(_goal.forward_speed)

		self.publish_guidance_data = True



if __name__ == '__main__':
	try:
		vtf_path_following = VtfPathFollowing()
		vtf_path_following.spin()

	except rospy.ROSInterruptException:
		pass
