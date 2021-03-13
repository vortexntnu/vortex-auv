#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import numpy as np

class PIDRegulator:
	"""
	A very basic 1D PID controller
	"""

	def __init__(self, p, i, d, sat):
		"""
		Initialize the PID controller by setting gains
		and saturation

		Args:
			p	  proportional gain
			i	  integral gain
			d	  derivative gain
			sat	  saturation limit
		"""

		self.p = p
		self.i = i
		self.d = d
		self.sat = sat

		self.integral = 0
		self.prev_err = 0
		self.prev_t = -1

	def __str__(self):
		"""
		Create a formatted string containing the controller gains
		and the saturation limit.
		"""

		msg = 'PID controller:'
		msg += '\n\tp=%f' % self.p
		msg += '\n\cd ti=%f' % self.i
		msg += '\n\td=%f' % self.d
		msg += '\n\tsat=%f' % self.sat
		return msg

	def regulate(self, err, t):
		"""
		Calculate the controller gain 

		Args:
			err	  the state error used to calculate controller gain (e)
			t	  the current time

		Returns:
			float:	The controller gain u
		"""

		derr_dt = 0.0
		dt = t - self.prev_t
		if self.prev_t > 0.0 and dt > 0.0:
			derr_dt = (err - self.prev_err)/dt
			self.integral += 0.5*(err + self.prev_err)*dt

		u = self.p*err + self.d*derr_dt + self.i*self.integral

		self.prev_err = err
		self.prev_t = t

		if (np.linalg.norm(u) > self.sat):
			# controller is in saturation: limit output, reset integral
			u = self.sat*u/np.linalg.norm(u)
			self.integral = 0.0

		return u
