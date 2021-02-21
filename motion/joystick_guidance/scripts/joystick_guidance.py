#!/usr/bin/env python
# Written by Jae Hyeong Hwang
# Copyright (c) 2021, Vortex NTNU.
# All rights reserved.

import rospy

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy

# to configure joystick environment, please refer to http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick


def nearest_list_value(value, ref_list):
	"""
	Returns the highest value in the list 'ref_list' that is
	less or equal to the given value 'value'.
	"""

	# Sorting the list in ascending order
	ref_list = sorted(ref_list, key = lambda x:float(x)) 

	for i in range(len(ref_list)):
		if i == 0 and value < ref_list[i]:
			value = 0
			break
		if value < ref_list[i]:
			# The first if-should prevent out of bounds
			value = ref_list[i - 1]
			break
	if value not in ref_list:
		# We know that we are given largest value in the list
		value = ref_list[-1]
	
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


        # Required input-values to generate valid signals
        limits = []
        self.max_value = pow(2, self.joystick_num_bit - 1)  # -1 due to only positive integers
        for i in range(1, self.num_ranges + 1):         
            limits.append((max_value / self.num_ranges) * i)
        self.limits = limits


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
			vector_length_square += pow(calculated_point(i), 2)

		# Scale the point to hold withing the designated area
		if vector_length_square < pow(self.min_point_range, 2):
			# Under the set limit. Setting current vector to 0
			vector_length_square *= 0
		elif vector_length_square >= pow(self.max_point_range, 2):
			# Over the set limit. Normalizing
			calculated_point /= sqrt(vector_length_square)

		return calculated_point


	def scale_force_vectors(self, joystick_msg):
        """
        Scales the thrust-vectors to
        """

        # Recovering the given forces
		scale_x_vec = joystick_msg.force.x
		scale_y_vec = joystick_msg.force.y
		scale_z_vec = joystick_msg.force.z

        # Using a try-catch to prevent out-of-bounds to become a large problem
        try:
            # Scale each force
			scale_x_vec = nearest_list_value(scale_x_vec, self.limits)
			scale_y_vec = nearest_list_value(scale_y_vec, self.limits)
			scale_z_vec = nearest_list_value(scale_z_vec, self.limits)

			return scale_x_vec, scale_y_vec, scale_z_vec
        
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

