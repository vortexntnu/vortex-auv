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
from autopilot.autopilot import AutopilotBackstepping, AutopilotPID
from reference_model.discrete_tustin import ReferenceModel

class LOS:

	def __init__(self):

		# update rate
		self.h = 0.05
		self.u = 0.0

		# current position
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0

		# previous waypoint
		self.x_k = 0.0
		self.y_k = 0.0

		# next waypoint
		self.x_kp1 = 0.0
		self.y_kp1 = 0.0

		# depth hold depth
		self.z_d = 0.0

		# desired speed
		self.speed = 0

		# sphere of acceptance
		self.R = 0.5

		# look-ahead distance
		Lpp = 0.7
		self.delta = 2.0*Lpp


	def updateState(self, x, y, z, u, v, w, psi, r, time):

		self.x = x
		self.y = y
		self.z = z

		self.u_dot = (u - self.u) / self.h
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

		self.flag = False

		rospy.init_node('los_path_following')
		self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback, queue_size=1) # 20hz
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)
		self.pub_desired = rospy.Publisher('/manta/los_desired', Odometry, queue_size=1)

		# constructor object
		self.los = LOS()
		self.PID = AutopilotPID()
		self.autopilot = AutopilotBackstepping()
		self.reference_model = ReferenceModel(np.array((0, 0)), self.los.h)

		# dynamic reconfigure
		self.config = {}
		self.srv_reconfigure = Server(AutopilotConfig, self.config_callback)

		"""
			action server guide
			https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/simple_action_server.py
		"""
		self.action_server = actionlib.SimpleActionServer(name='los_path', ActionSpec=LosPathFollowingAction, auto_start=False)
		self.action_server.register_goal_callback(self.goalCB)
		self.action_server.register_preempt_callback(self.preemptCB)
		self.action_server.start()

	def callback(self, msg): 

		# update current position
		self.psi = self.los.quat2euler(msg)

		# update position and velocities
		self.los.updateState(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
							 msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
							 self.psi, msg.twist.twist.angular.z, msg.header.stamp.to_sec())

		# update current goal
		self.psi_d = self.los.lookaheadBasedSteering()

		if self.flag is True:

			# reference model
			x_smooth = self.reference_model.discreteTustinMSD(np.array((self.los.speed, self.psi_d)))

			print("\n raw psi_d: ")
			print(self.psi_d)
			print(" vs psi_d_smooth: ")
			print(x_smooth[2])
			print(" vs psi")
			print(self.psi)

			u_d = x_smooth[0]
			u_d_dot = x_smooth[1]
			psi_d = x_smooth[2]
			r_d = x_smooth[3]
			r_d_dot = x_smooth[4]

			los_msg = Odometry()
			los_msg.header.stamp = rospy.Time.now()
			quat_d = quaternion_from_euler(0, 0, psi_d)
			los_msg.pose.pose.position.z = msg.pose.pose.position.z
			los_msg.pose.pose.orientation.x = quat_d[0]
			los_msg.pose.pose.orientation.y = quat_d[1]
			los_msg.pose.pose.orientation.z = quat_d[2]
			los_msg.pose.pose.orientation.w = quat_d[3]
			los_msg.twist.twist.linear.x = u_d
			los_msg.twist.twist.angular.z = r_d
			self.pub_desired.publish(los_msg)

			# control force
			tau_d = self.autopilot.backstepping.controlLaw(self.los.u, self.los.u_dot, u_d, u_d_dot, self.los.v, self.psi, psi_d, self.los.r, r_d, r_d_dot)
			tau_depth_hold = self.PID.depthController(self.los.z_d, self.los.z, self.los.t)

			# add speed controllers here
			thrust_msg = Wrench()
			if tau_d[0] > 0.0:
				thrust_msg.force.x = tau_d[0]

			thrust_msg.force.y = tau_d[1]
			thrust_msg.force.z = tau_depth_hold
			thrust_msg.torque.z = tau_d[2] # 2.0*self.error_ENU

			# write to thrusters
			self.pub_thrust.publish(thrust_msg)


			# check if action goal succeeded
			self.statusActionGoal()

	def statusActionGoal(self):

		# feedback
		self._feedback.distanceToGoal = self.los.distance()
		self.action_server.publish_feedback(self._feedback)

		# succeeded
		if self.los.sphereOfAcceptance():
			self._result.terminalSector = True
			self.action_server.set_succeeded(self._result, text="goal completed")
			self.flag = False

	def preemptCB(self):
		# check that preempt has not been requested by the client
		if self.action_server.is_preempt_requested():
			rospy.loginfo("Preempted requested by los path client")
			self.action_server.set_preempted()
			self.flag = False

	def goalCB(self):

		self.flag = True
		_goal = self.action_server.accept_new_goal()

		# set goal
		self.los.x_k = self.los.x
		self.los.y_k = self.los.y
		self.los.x_kp1 = _goal.next_waypoint.x
		self.los.y_kp1 = _goal.next_waypoint.y

		# forward speed
		self.los.speed = _goal.forward_speed.linear.x

		# depth hold
		self.los.z_d = _goal.desired_depth.z

		# sphere of acceptance
		self.los.R = _goal.sphereOfAcceptance

		self.reference_model = ReferenceModel(np.array((self.los.u, self.los.psi)), self.los.h)


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
