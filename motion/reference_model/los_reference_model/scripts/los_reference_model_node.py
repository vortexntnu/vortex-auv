#!/usr/bin/python3

import rospy
import numpy as np
import math

from discrete_tustin import ReferenceModel

from vortex_msgs.msg import GuidanceData


class LOSReferenceModelNode():
	"""
	This is the ROS wrapper class for the reference model class. 
	
	Nodes created:
		los_reference_model

	Subscribes to:
		/guidance/los_data
	
	Publishes to:
		/reference_model/los_data
	"""
	def __init__(self):
		# Update rate
		self.h = 0.05
		self.u = 0.0

		# ROS node init
		rospy.init_node('rm_los')

		# Subscribers
		self.guidance_data_sub = rospy.Subscriber('/guidance/los_data', GuidanceData, self.guidanceDataCallback, queue_size=1)

		# Publishers
		self.rm_los_data_pub = rospy.Publisher('/reference_model/los_data', GuidanceData, queue_size=1)

		# RM object
		self.reference_model = ReferenceModel(np.array((0, 0)), self.h)

	def fixHeadingWrapping(self, u, psi, psi_ref, speed):
		"""
		The heading angle is obtained by the use of an arctangent
		function, which is discontinuous at -pi and pi. This can 
		be problematic when the heading angle is fed into the
		reference model. This function fixes this problem by
		wrapping the angles around by 2pi.
		"""

		e = psi - psi_ref
		if e < -math.pi:
			psi_ref = psi_ref - 2*math.pi
		if e > math.pi:
			psi_ref = psi_ref + 2*math.pi


		# Reference Model
		x_d = self.reference_model.discreteTustinMSD(np.array((speed, psi_ref)))
		psi_d = x_d[2]

		e = psi - psi_d
		if e > math.pi:
			psi_d = psi_d - 2*math.pi
			self.reference_model = ReferenceModel(np.array((u, psi)), self.h)
			x_d = self.reference_model.discreteTustinMSD(np.array((speed, psi_d)))
		if e < -math.pi:
			psi_d = psi_d + 2*math.pi
			self.reference_model = ReferenceModel(np.array((u, psi)), self.h)
			x_d = self.reference_model.discreteTustinMSD(np.array((speed, psi_d)))
		return x_d

	def guidanceDataCallback(self, msg):
		"""
		Gets topic msg from LOS guidance module, applies discrete Tustin transform,
		publishes data from guidance and the calculated data to LOS controller.
		"""

		# Guidance data sub
		u = msg.u
		psi = msg.psi
		psi_ref = msg.psi_ref
		speed = msg.speed 

		# Reference model calc
		"""
		Wrapping would have been avoided by using quaternions instead of Euler angles
		if you don't care about wrapping, use this instead:

		x_d = self.reference_model.discreteTustinMSD(np.array((self.los.speed,psi_d)))
		"""
		x_d = self.fixHeadingWrapping(u, psi, psi_ref, speed)

		# Reference Model data pub

		msg.u_d = x_d[0]
		msg.u_d_dot = x_d[1]
		msg.psi_d = x_d[2]
		msg.r_d = x_d[3]
		msg.r_d_dot = x_d[4]

		self.rm_los_data_pub.publish(msg)

if __name__ == '__main__':
	try:
		rm_los_node = LOSReferenceModelNode()

		rospy.spin()
	except rospy.ROSInterruptException:
		pass
