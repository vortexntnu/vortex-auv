#!/usr/bin/env python
import rospy
import numpy as np
import math
from vortex_msgs.msg import PropulsionCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# modules included in this package
from PID import PIDRegulator

class LOS:

	def __init__(self):

		self.waypoint_k = PoseStamped()
		self.waypoint_kp1 = PoseStamped()

		# current position
		self.x = 0.0
		self.y = 0.0

		# previous waypoint
		self.x_k = 5.0
		self.y_k = 5.0

		# next waypoint
		self.x_kp1 = 10.0
		self.y_kp1 = -2.0

		# los target
		self.x_los = 0
		self.y_los = 0

		# sphere of acceptance
		self.R = 0.5


	def updatePosition(self, x, y, z, u, psi, r):

		self.x = x
		self.y = y
		self.z = z
		self.u = u
		self.psi = psi
		self.r = r

	def setWayPoints(self, x_k, y_k, x_kp1, y_kp1):

		# previous waypoint
		self.x_k = x_k
		self.y_k = y_k

		# next waypoint
		self.x_kp1 = x_kp1
		self.y_kp1 = y_kp1

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

		# look-ahead distance
		Lpp = 0.7
		delta = 2.0*Lpp

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
		print('\n cross-track error: ')
		print(self.e)

		# path-tangential angle (eq 10.73 Fossen)
		self.chi_p = self.alpha

		# velocity-path relative angle (eq 10.74 Fossen)
		self.chi_r = np.arctan(-self.e / delta)

		# desired heading angle
		self.chi_d = self.chi_p + self.chi_r

		return self.chi_d

class LosPathFollowing(object):

	def __init__(self):
		rospy.init_node('los_path_following')
		self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback, queue_size=1) # 20hz
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)

		# constructor object
		self.los = LOS()


	def headingController(self, psi, psi_d, r, r_d):
		# PIDRegulator(p, i, d, sat)
		self.pid_heading = PIDRegulator(1, 0, 0, 1)
		


	def callback(self, msg):

		# time
		self.t = msg.header.stamp.to_sec()

		# update current position

		self.psi = self.los.quat2euler(msg)
		self.los.updatePosition(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
								msg.twist.twist.linear.x, self.psi, msg.twist.twist.angular.z)

		# update current goal
		self.psi_d = self.los.lookaheadBasedSteering()
		#print('\n self heading_d: ')
		#print(self.heading_d)

		self.error_NED = (self.psi - self.psi_d)
		self.error_ENU = -self.error_NED
		print('\n error heading: ')
		print(self.error_ENU)
		self.norm_error = self.error_ENU/(abs(self.error_ENU)+1)
		print('\n error norm: ')
		print(self.norm_error)


		thrust_msg = Wrench()

		# add speed controllers here
		thrust_msg.force.x = 1.0
		thrust_msg.torque.z = 2.0*self.error_ENU
		#print(thrust_msg)
		self.pub_thrust.publish(thrust_msg)


if __name__ == '__main__':
	try:
		los_path_following = LosPathFollowing()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass