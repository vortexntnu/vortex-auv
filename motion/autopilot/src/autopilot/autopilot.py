#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.

import rospy

from geometry_msgs.msg import Wrench
from vortex_msgs.msg import GuidanceData

from PID import PIDRegulator
from backstepping import BacksteppingDesign, BacksteppingControl


class AutopilotPID:

	def __init__(self):
		# PIDRegulator(p, i, d, sat)
		self.pid = PIDRegulator(25, 0.024, 3.5, 5.0)

	def updateGains(self, p, i, d, sat):

		self.pid.p = p
		self.pid.i = i
		self.pid.d = d
		self.pid.sat = sat

	def headingController(self, psi_d, psi, t):

		# error ENU
		e_rot = psi_d - psi

		# regulate(err, t)
		tau = self.pid.regulate(e_rot, t)

		return tau

	def depthController(self, z_d, z, t):

		e = z_d - z

		tau = self.pid.regulate(e, t)

		return tau

class CameraPID:

	"""
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

	def __init__(self):

		self.sway = PIDRegulator(0.01, 0.0001, 0.0, 7.5)
		#self.heading = PIDRegulator(0.02, 0.0, 0.0, 0.15)
		self.heading = PIDRegulator(25,0.002, 0.0, 100)
		self.depth = PIDRegulator(25, 0.024, 3.5, 5.0)
		self.speed = PIDRegulator(25, 0.024, 3.5, 5.0)

	def swayController(self, px_d, px, t):

		# error
		e = px_d - px

		tau = self.sway.regulate(e, t)

		return tau

	def depthController(self, z_d, z, t):

		e = z_d - z

		tau = self.depth.regulate(e, t)

		return tau


	def speedController(self, u_d, u, t):

		e = u_d - u

		tau = self.speed.regulate(e, t)

		return tau

	def headingController(self, psi_d, psi, t):

		# error ENU
		e_rot = psi_d - psi

		# regulate(err, t)
		tau = self.heading.regulate(e_rot, t)

		return tau

class AutopilotBackstepping:
	"""
	Wrapper for the backstepping controller.

	Attributes:
		backstepping	The controller
	"""

	def __init__(self):
												# 0.75, 30, 12, 2.5
		self.backstepping = BacksteppingControl(3.75, 45.0, 28.0, 10.5)

	def updateGains(self, c, k1, k2, k3):

		pass


class Autopilot:
	"""
	The ROS wrapper class for the Autopilot. The autopilot is made up
	of a PID and a backstepping controller, and is mainly used in
	conjunction with the LOS guidance system.

	Nodes created:
		autopilot

	Subscribes to:
		/manta/guidance_data

	Publishes to:
		/manta/thruster_manager/input

	"""

	def __init__(self):

		rospy.init_node('autopilot')

		# Create controllers
		self.Backstepping = AutopilotBackstepping()
		self.PID = AutopilotPID()

	
		# Subscribers
		self.sub_guidance = rospy.Subscriber('/manta/LOS_guidance_data', GuidanceData, self.guidance_data_callback, queue_size=1)

		# Publishers
		self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)


	def guidance_data_callback(self, msg):
		"""
		Handle guidance data whenever it is published by calculating
		a control vector based on the given data.

		Args:
			msg:	The guidance data message
		"""

		# Control forces
		tau_d = self.Backstepping.backstepping.controlLaw(
				u,
				u_dot,
				u_d,
				u_d_dot,
				v,
				psi, psi_d, 
				r,
				r_d,
				r_d_dot)

		tau_depth_hold = self.PID.depthController(z_d, z, t)

		# add speed controllers here

		thrust_msg = Wrench()

		# Thrust message forces and torque
		if tau_d[0] > 0.0:
			thrust_msg.force.x = tau_d[0]

		thrust_msg.force.y = tau_d[1]
		thrust_msg.force.z = tau_depth_hold

		thrust_msg.torque.z = tau_d[2]

		# Publish the thrust message to /manta/thruster_manager/input
		self.pub_thrust.publish(thrust_msg)
		
		



if __name__ == '__main__':
	try:
		autopilot = Autopilot()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
