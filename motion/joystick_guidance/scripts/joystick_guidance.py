#!/usr/bin/env python
# Written by Jae Hyeong Hwang
# Copyright (c) 2021, Vortex NTNU.
# All rights reserved.

import rospy

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from math import sqrt

# to configure joystick environment, please refer to http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick


def nearest_list_value(param_list, ref_list, decimal_list):
	"""
	Returns the highest value in the list 'ref_list' that is
	less or equal to the given value 'value'.
	"""

	# Extracting the value and deadzone
	value = param_list[0]
	deadzone = param_list[1]

	# Sorting the list in ascending order
	ref_list = sorted(ref_list, key = lambda x:float(x)) 

	# Using a temp_value to keep track of sign
	temp_value = 0

	for i in range(len(ref_list)):
		if value < ref_list[i]:
			temp_value = decimal_list[i]
			break

	# We allow maximum thrust when it is given below a certain threshold
	if value >= ref_list[-1] - deadzone:
		# We know that we are given largest value in the list
		temp_value = decimal_list[-1]

	# Correcting for negative sign
	if value < 0:
		temp_value *= -1 

	value = temp_value
	return value


class JoystickGuidanceNode():

	def __init__(self):

		rospy.init_node('joystick_guidance')

		self.sub = rospy.Subscriber('/guidance/joystick_data', Joy, self.callback, queue_size=1)
		self.pub = rospy.Publisher('/auv/thruster_manager/input', Wrench, queue_size=1)

		self.surge 	= 0
		self.sway 	= 1
		self.heave 	= 2
		self.roll 	= 3 
		self.pitch 	= 4 
		self.yaw 	= 5 

		self.min_point_range = 0.2                          # Minimum allowed distance from the UUV to the point
		self.max_point_range = 1                            # Maximum allowed distance from the UUV to the calculated point
		self.num_ranges = 5                                 # Number of ranges the input is scaled by
		self.joystick_num_bit = 16                          # Resolution on the joystick
		self.deadzone = 1000								# Deadzone on upper input

		# Required input-values to generate valid signals
		limits = []
		self.max_value = pow(2, self.joystick_num_bit - 1)  # -1 due to only positive integers
		for i in range(1, self.num_ranges + 1):         
			limits.append((self.max_value / self.num_ranges) * i)
		self.limits = limits


		# Calculated input in decimal
		decimal_list = []
		for i in range(0, num_ranges + 1):
			decimal_list.append((max_point_range / num_ranges) * i)
		self.decimal_list = decimal_list


	def callback(self, msg):

		joystick_msg = Wrench()
		joystick_msg.force.x  = msg.axes[self.surge]
		joystick_msg.force.y  = msg.axes[self.sway]
		joystick_msg.force.z  = msg.axes[self.heave]
		joystick_msg.torque.x = msg.axes[self.roll]
		joystick_msg.torque.y = msg.axes[self.pitch]
		joystick_msg.torque.z = msg.axes[self.yaw]

		self.pub.publish(joystick_msg)

	def calculate_joystick_point(self, joystick_msg):
		"""
		Calculates a point in the local frame based on the given thrust-vectors 
		from joystick 

		If required it must be converted into the global reference-frame
		"""

		# Scaling the force to get a linearized model
		scaled_force_x, scaled_force_y, scaled_force_z = scale_force_vectors(joystick_msg)

		# TODO Change this to the required point-type size
		calculated_point = [scaled_force_x, scaled_force_y, scaled_force_z]

		# Calculating the length of the vectors
		vector_length_square = 0
		for i in range(len(calculated_point)):
			vector_length_square += pow(calculated_point[i], 2)

		# Normalizing if exceeding the max_point_range
		if vector_length_square >= pow(max_point_range, 2):
			# Over the set limit. Normalizing
			vector_length = sqrt(vector_length_square)
			calculated_point = [val / vector_length for val in calculated_point]

		return calculated_point


	def scale_force_vectors(self, joystick_msg):
		"""
		Scales the thrust-vectors to
		"""

		# Recovering the given forces
		force_x = joystick_msg.force.x
		force_y = joystick_msg.force.y
		force_z = joystick_msg.force.z

		# Using a try-catch to prevent out-of-bounds to become a large problem
		try:
			# Set up a list with param
			param_list_x = [force_x, self.deadzone]
			param_list_y = [force_y, self.deadzone]
			param_list_z = [force_z, self.deadzone]

			# Scale each force
			force_x = nearest_list_value(param_list_x, self.limits, self.decimal_list)
			force_y = nearest_list_value(param_list_y, self.limits, self.decimal_list)
			force_z = nearest_list_value(param_list_z, self.limits, self.decimal_list)

			return force_x, force_y, force_z

		except Exception as e:
			rospy.logerr(e)
			
			# Return a standard response if an error occurs
			return 0, 0, 0
 

if __name__ == '__main__':

	try:
		joystick_guidance = JoystickGuidanceNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

