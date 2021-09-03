#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
import numpy as np
import time
class ReferenceModel:

	def __init__(self, x, h):
		
		""" 
		x = []
		x_d[k] = b0*x_ref*[k] + b1*x_ref*[k - 1] + b2*x_ref*[k - 2] - a1*x_d*[k-1] - a_2*x_d*[k - 2]
		"""
		self.h = h
		# determines position of poles
		self.a0 = 1.0
		self.a1 = -1.9312
		self.a2 = 0.9324

		# determine zeros
		self.b0 = 2.9581e-04
		self.b1 = 5.9161e-04
		self.b2 = 2.9581e-04

		# set intial values
		self.x_ref_prev_prev = x	# 2nd-to-last raw input value
		self.x_ref_prev = x			# last raw input value
		self.x_ref = x				# current raw input value
		self.x_d_prev_prev = x		# 2nd-to-last filtered (output) value
		self.x_d_prev = x			# last filtered (output) value
		self.x_d = x				# current filtered (output value)

		# set initial values for derivates
		self.x_d_dot = 0
		self.x_d_dot_prev = 0 
		self.x_d_dot_dot = 0

	def resetFilter(self, x):

		# set intial values
		self.x_ref_prev_prev = x	# 2nd-to-last raw input value
		self.x_ref_prev = x			# last raw input value
		self.x_ref = x				# current raw input value
		self.x_d_prev_prev = x		# 2nd-to-last filtered (output) value
		self.x_d_prev = x			# last filtered (output) value
		self.x_d = x				# current filtered (output value)

		# set initial values for derivates
		self.x_d_dot = 0
		self.x_d_dot_prev = 0 
		self.x_d_dot_dot = 0

	def computeDerivatives(self):
		# new setpoint
		# [u_d_dot, r_d]
		self.x_d_dot = (self.x_d - self.x_d_prev)/self.h
		
		# [u_d_dot_dot, r_d_dot]
		self.x_d_dot_dot = (self.x_d_dot - self.x_d_dot_prev)/self.h

		# update prev
		self.x_d_dot_prev = self.x_d_dot

		# [u_d, u_d_dot, psi_d, r_d, r_d_dot]
		x_output = 	np.array((self.x_d[0],
							  self.x_d_dot[0],
							  self.x_d[1],
							  self.x_d_dot[1],
							  self.x_d_dot_dot[1]))
		return x_output

	def discreteTustinMSD(self, x_ref):

		self.x_ref = x_ref

		# new set point
		self.x_d = self.b0*self.x_ref + self.b1*self.x_ref_prev + self.b2*self.x_ref_prev_prev - self.a1 * self.x_d_prev - self.a2 * self.x_d_prev_prev

		# include velocities and accelerations
		self.x_output = self.computeDerivatives()

		# update
		self.x_ref_prev_prev = self.x_ref_prev
		self.x_ref_prev = self.x_ref
		self.x_d_prev_prev = self.x_d_prev
		self.x_d_prev = self.x_d

		return self.x_output

if __name__ == '__main__':

	x = np.array((0.5, 0.0))
	x_ref = np.array((2.0,1.57))

	rm = ReferenceModel(x, 0.05)
	count = 0

	while (count < 400):
		time.sleep(.05)
		xd = rm.discreteTustinMSD(x_ref)
		print(xd)
