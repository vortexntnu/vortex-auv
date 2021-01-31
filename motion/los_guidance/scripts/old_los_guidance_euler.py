#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Documented by Christopher Strom and Jae Hyeong Hwang
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
import numpy as np
import math
from vortex_msgs.msg import PropulsionCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# dynamic reconfigure
from dynamic_reconfigure.server import Server
from los_guidance.cfg import LOSControllerConfig

# action message
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback

# modules included in this package
from los_controller.los_controller import LOSControllerPID
from reference_model.discrete_tustin import ReferenceModel

class LOS:
	"""
	The Line-Of-Sight guidance class, with an imported controller.

	Physical attributes referenced in the class:
	x, y, z: 	surge, sway, heave (position)
	u, v, w:	surge, sway, heave (velocity)

	alpha:	The path-tangential angle
  	psi:	Heading angle required to reach the LOS intersection
	  		point.

	R: sphere of acceptance. If the AUV is inside the sphere
	   defined by this radius and the setpoint, it will be
	   considered to have reached the setpoint.
	"""

	def __init__(self):

		# current position
		self.x = 0.0
		self.y = 0.0

		# previous waypoint
		self.x_k = 0.0
		self.y_k = 0.0

		# next waypoint
		self.x_kp1 = 0.0
		self.y_kp1 = 0.0

		# sphere of acceptance
		self.R = 0.5

		# look-ahead distance
		Lpp = 0.7
		self.delta = 1.0*Lpp


	def updateState(self, x, y, z, u, v, w, psi, r, time):
		"""
		Update all state values contained in the LOS class.

		Args:
			x	  Surge; position in the direction of the x-axis.
			y	  Sway;  position in the direction of the y-axis.
			z	  Heave; position in the direction of the z-axis.

			u	  Body fixed velocity in the x-direction.
			v	  Body fixed velocity in the y-direction.
			w	  Body fixed velocity in the z-direction.

  			psi	  Heading angle required to reach the LOS intersection
			  	  point.
			r	  current angular velocity around the body-fixed z-axis
			time  A double with the current time
		"""

		# Update position
		self.x = x
		self.y = y
		self.z = z

		# Update velocities
		self.u = u
		self.v = v
		self.w = w

		self.psi = psi
		self.r = r
		self.t = time

	def setWayPoints(self, x_k, y_k, x_kp1, y_kp1):
		"""
		Set the previous and next waypoints

		Args:
			x_k     x-component of the previous waypoint
			y_k     y-component of the previous waypoint

			x_kp1	x-component of the next waypoint
			y_kp1	y-component of the next waypoint
		"""

		# previous waypoint
		self.x_k = x_k
		self.y_k = y_k

		# next waypoint
		self.x_kp1 = x_kp1
		self.y_kp1 = y_kp1

	def distance(self):
		"""
		Calculate straight line distance (2D) between the
		current position and the setpoint position.
		"""

		return np.sqrt((self.x_kp1 - self.x)**2 + 
					   (self.y_kp1 - self.y)**2 )

	def sphereOfAcceptance(self):
		"""
		The sphere of acceptance is a sphere around the setpoint.
		If the AUV is inside this sphere, it will be considered
		as having reached the setpoint.

		Returns:
			bool:	True if the current position is less than the
					radius of the sphere of acceptance. False otherwise
		"""

		return self.distance() < self.R

	def getEpsilonVector(self):
		"""
		Calculate the epsilon vector, which is the vector
		that contains the coordinates of the AUV in the 
		path-fixed reference frame for a straight line going
		from the reference point to the target position.

		Returns:
			float: The calculated epsilon vector

		"""

		alpha = self.alpha

		# rotation matrix
		R = np.array(( (np.cos(alpha), -np.sin(alpha)),
					   (np.sin(alpha),  np.cos(alpha)) ))

		# transpose
		R_T = np.transpose(R)

		# position vector
		p_t = np.array((self.x, self.y))
		p_k = np.array((self.x_k, self.y_k))

		# epsilon (eq 10.56 Fossen)
		epsilon = R_T.dot(p_t - p_k) 

		return epsilon

	def quat2euler(self,msg):
		"""
		Calculate roll, pitch and yaw from the orientation
		quaternion with the axis sequence xyzw

		Args:
			msg		A nav_msgs/Odometry message

		Returns:
			float: The euler yaw angle calculated from the msg argument
		"""

		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll,pitch,yaw) = euler_from_quaternion(orientation_list)

		return yaw

	def lookaheadBasedSteering(self):
		"""
		Calculate the desired heading angle. This angle is
		the sum of the path-tangential angle and the velocity-
		path relative angle.

		Returns:
			float: The desired heading angle chi_d
		"""

		# straight-line path segment
		self.y_delta = self.y_kp1 - self.y_k
		self.x_delta = self.x_kp1 - self.x_k

		# angle
		self.alpha = np.arctan2(self.y_delta, self.x_delta)

		# rotation matrix
		epsilon = self.getEpsilonVector()

		# along track distance
		self.s = epsilon[0]

		# cross-track error
		self.e = epsilon[1]
		#print('\n cross-track error: ')
		#print(self.e)

		# path-tangential angle (eq 10.73 Fossen)
		self.chi_p = self.alpha

		# velocity-path relative angle (eq 10.74 Fossen)
		self.chi_r = np.arctan(-self.e / self.delta)

		# desired heading angle
		self.chi_d = self.chi_p + self.chi_r

		return self.chi_d

class LosPathFollowing(object):
	"""
	This is the ROS wrapper class for the LOS class. 

	Attributes:
		_feedback	A vortex_msgs action that contains the distance to goal
		_result		A vortex_msgs action, true if a goal is set within the
					sphereof acceptance, false if not
	
	Nodes created:
		los_path_following

	Subscribes to:
		/odometry/filtered
	
	Publishes to:
		/manta/thruster_manager/input
		/manta/los_desired
	
	"""

	# create messages that are used to send feedback/result
	_feedback = LosPathFollowingFeedback()
	_result = LosPathFollowingResult()

	def __init__(self):
		"""
		To initialize the ROS wrapper, the node, subscribers
		and publishers are set up. The high-level guidance and
		controller objects are also intialized. Lastly, dynamic
		reconfigure and action servers are set up.
		"""

		"""
		A flag to indicate whether or not a goal has not been reached.
		True means that a goal is in progress of being completed.
		False means that a goal has been completed (or not started
		with any goal)
		"""
		self.flag = False

		rospy.init_node('los_path_following')
		self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback, queue_size=1) # 20hz
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)

		# constructor object
		self.los = LOS()
		self.los_controller = LOSControllerPID()
		self.reference_model = ReferenceModel(np.array((0, 0)), 0.05)

		# dynamic reconfigure
		self.config = {}
		self.srv_reconfigure = Server(LOSControllerConfig, self.config_callback)

		"""
			action server guide
			https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/simple_action_server.py
		"""
		self.action_server = actionlib.SimpleActionServer(name='los_path', ActionSpec=LosPathFollowingAction, auto_start=False)
		self.action_server.register_goal_callback(self.goalCB)
		self.action_server.register_preempt_callback(self.preemptCB)
		self.action_server.start()

	def callback(self, msg): 
		"""
		The callback used in the subscribed topic /odometry/filtered.
		When called, position and velocity states are updated, and 
		a new current goal is set.

		Args:
			msg		A nav_msgs/Odometry ROS message type
		"""

		# update current position
		self.psi = self.los.quat2euler(msg)

		# update position and velocities
		self.los.updateState(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
							 msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
							 self.psi, msg.twist.twist.angular.z, msg.header.stamp.to_sec())

		# update current goal
		self.psi_d = self.los.lookaheadBasedSteering()

		# reference model
		x_smooth = self.reference_model.discreteTustinMSD(np.array((2.0, self.psi_d)))
		print(x_smooth)
		
		# control force
		tau_d_heading = self.los_controller.headingController(self.psi_d, self.psi, self.los.t)

		# add speed controllers here
		thrust_msg = Wrench()
		thrust_msg.force.x = 1.0
		thrust_msg.torque.z = tau_d_heading # 2.0*self.error_ENU

		if self.flag is True:
			# write to thrusters
			self.pub_thrust.publish(thrust_msg)

			# check if action goal succeeded
			self.statusActionGoal()

	def statusActionGoal(self):
		"""
		Publish the current distance to target and check if the current
		position is within the sphere of acceptance. If it is, the 
		attribute _result can be set to true, and the flag to false.
		"""

		# feedback
		self._feedback.distanceToGoal = self.los.distance()
		self.action_server.publish_feedback(self._feedback)

		# succeeded
		if self.los.sphereOfAcceptance():
			self._result.terminalSector = True
			self.action_server.set_succeeded(self._result)

	def preemptCB(self):
		"""
		The preempt callback for the action server.
		"""

		# check that preempt has not been requested by the client
		if self.action_server.is_preempt_requested():
			rospy.loginfo("Preempted requested by los path client")
			self.action_server.set_preempted()

	def goalCB(self):
		"""
		The goal callback for the action server.
		"""

		self.flag = True
		_goal = self.action_server.accept_new_goal()

		# set goal
		self.los.x_k = _goal.prev_waypoint.x
		self.los.y_k = _goal.prev_waypoint.y
		self.los.x_kp1 = _goal.next_waypoint.x
		self.los.y_kp1 = _goal.next_waypoint.y

		# forward speed
		self.speed = _goal.forward_speed.linear.x

		# sphere of acceptance
		self.los.R = _goal.sphereOfAcceptance


	def config_callback(self, config, level):
		"""
		Handle updated configuration values.
		
		Args:
			config	The dynamic reconfigure server's config
			level	Ununsed variable

		Returns:
			The updated config argument.
		"""

		# Config has changed, reset PID controllers
		rospy.loginfo("""Reconfigure Request: {delta}, {p_rot}, {i_rot}, {d_rot}, {sat_rot} """.format(**config))
        
		# update look-ahead distance
		self.los.delta = config['delta']

		# self.pid_lin = PIDRegulator(config['pos_p'], config['pos_i'], config['pos_d'], config['pos_sat'])
		self.los_controller.updateGains(config['p_rot'], config['i_rot'], config['d_rot'], config['sat_rot'])

		# update config
		self.config = config

		return config


if __name__ == '__main__':
	try:
		los_path_following = LosPathFollowing()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
