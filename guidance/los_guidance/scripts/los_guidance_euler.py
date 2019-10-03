#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
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
from los_guidance.cfg import AutopilotConfig

# action message
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback

# modules included in this package
from autopilot.autopilot import Autopilot

class LOS:

	def __init__(self):

		self.waypoint_k = PoseStamped()
		self.waypoint_kp1 = PoseStamped()

		# current position
		self.x = 0.0
		self.y = 0.0

		# previous waypoint
		self.x_k = -5.0
		self.y_k = 5.0

		# next waypoint
		self.x_kp1 = 10.0
		self.y_kp1 = -2.0

		# los target
		self.x_los = 0
		self.y_los = 0

		# sphere of acceptance
		self.R = 0.5

		# look-ahead distance
		Lpp = 0.7
		self.delta = 1.0*Lpp


	def updateState(self, x, y, z, u, v, w, psi, r, time):

		self.x = x
		self.y = y
		self.z = z
		self.u = u
		self.v = v
		self.w = w
		self.psi = psi
		self.r = r
		self.t = time

	def setWayPoints(self, x_k, y_k, x_kp1, y_kp1):

		# previous waypoint
		self.x_k = x_k
		self.y_k = y_k

		# next waypoint
		self.x_kp1 = x_kp1
		self.y_kp1 = y_kp1

	def distance(self):
		return np.sqrt((self.x_kp1 - self.x)**2 + 
					   (self.y_kp1 - self.y)**2 )

	def sphereOfAcceptance(self):
		return self.distance() < self.R

	def getEpsilonVector(self):

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
		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll,pitch,yaw) = euler_from_quaternion(orientation_list)

		return yaw

	def lookaheadBasedSteering(self):

		# straight-line path segment
		self.y_delta = self.y_kp1 - self.y_k
		self.x_delta = self.x_kp1 - self.x_k

		# angle
		self.alpha = np.arctan2(self.y_delta, self.x_delta)

		#if self.alpha < 0:
		#	self.alpha = self.alpha + 2*math.pi

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

	# create messages that are used to send feedback/result
	_feedback = LosPathFollowingFeedback()
	_result = LosPathFollowingResult()

	def __init__(self):
		rospy.init_node('los_path_following')
		self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback, queue_size=1) # 20hz
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)

		# constructor object
		self.los = LOS()
		self.autopilot = Autopilot()

		# dynamic reconfigure
		self.config = {}
		self.srv_reconfigure = Server(AutopilotConfig, self.config_callback)

		"""
			action server
			https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/simple_action_server.py
		"""

		self.action_server = actionlib.SimpleActionServer(name='los_path', ActionSpec=LosPathFollowingAction, auto_start=False)
		#self.action_server.register_goal_callback(self.goalCB)
		#self.action_server.register_preempt_callback(self.preemptCB)
		#self.action_server.start()
		#self.goalAccepted = False

	def callback(self, msg): 

		# update current position
		self.psi = self.los.quat2euler(msg)

		# update position and velocities
		self.los.updateState(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
							 msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
							 self.psi, msg.twist.twist.angular.z, msg.header.stamp.to_sec())

		# update current goal
		self.psi_d = self.los.lookaheadBasedSteering()
		
		# control force
		tau_d_heading = self.autopilot.headingController(self.psi_d, self.psi, self.los.t)

		# add speed controllers here
		thrust_msg = Wrench()
		thrust_msg.force.x = 1.0
		thrust_msg.torque.z = tau_d_heading # 2.0*self.error_ENU

		#print(thrust_msg)
		self.pub_thrust.publish(thrust_msg)

		# check if action goal succeeded
		#if self.goalAccepted:
		#	self.statusActionGoal()

	def statusActionGoal(self):

		# feedback
		self._feedback.distanceToGoal = 2.0 #self.los.distance()
		self.action_server.publish_feedback(self._feedback)

		# succeeded
		if self.los.sphereOfAcceptance():
			_result = True
			self.action_server.set_succeeded(_result)

	def preemptCB(self):
		# check that preempt has not been requested by the client
		if self.action_server.is_preempt_requested():
			rospy.loginfo("Preempted requested by los path client")
			self.action_server.set_preempted()

	def goalCB(self):

		self.goalAccepted = True
		_goal = self.action_server.accept_new_goal()

		# set goal
		self.los.x_k = _goal.prev_waypoint.x
		self.los.y_k = _goal.prev_waypoint.y
		self.los.x_kp1 = _goal.prev_waypoint.x
		self.los.y_kp1 = _goal.prev_waypoint.y

		# forward speed
		self.speed = _goal.forward_speed.linear.x

		# sphere of acceptance
		self.los.R = _goal.sphereOfAcceptance


	def config_callback(self, config, level):
		"""Handle updated configuration values."""
		# Config has changed, reset PID controllers

		rospy.loginfo("""Reconfigure Request: {delta}, {p_rot}, {i_rot}, {d_rot}, {sat_rot} """.format(**config))
        
		# update look-ahead distance
		self.los.delta = config['delta']

		# self.pid_lin = PIDRegulator(config['pos_p'], config['pos_i'], config['pos_d'], config['pos_sat'])
		self.autopilot.updateGains(config['p_rot'], config['i_rot'], config['d_rot'], config['sat_rot'])

		# update config
		self.config = config

		return config


if __name__ == '__main__':
	try:
		los_path_following = LosPathFollowing()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
