#!/usr/bin/env python
# Written by Jae Hyeong Hwang
# Copyright (c) 2021, Vortex NTNU.
# All rights reserved.

import rospy

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy

# to configure joystick environment, please refer to http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

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

	def calculate_joystick_point(joystick_msg):
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
			vector_length_square *= 0
		elif vector_length_square >= pow(self.max_point_range, 2):
			calculated_point /= vector_length_square

		return calculated_point


	def scale_force_vectors(joystick_msg):
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
            for i in range(len(self.limits)):
                if i == 0 and scale_x_vec < self.limits[i]:
                    scale_x_vec = 0
                    break
                if scale_x_vec < self.limits[i]:
                    # The first if-should prevent out of bounds
                    scale_x_vec = self.limits[i - 1]
                    break
            if scale_x_vec not in self.limits:
                # We know that we are given max force
                scale_x_vec = self.limits[-1]


            for i in range(len(self.limits)):
                if i == 0 and scale_y_vec < self.limits[i]:
                    scale_y_vec = 0
                    break
                if scale_y_vec < self.limits[i]:
                    # The first if-should prevent out of bounds
                    scale_y_vec = self.limits[i - 1]
                    break
            if scale_y_vec not in self.limits:
                # We know that we are given max force
                scale_y_vec = self.limits[-1]

            
            for i in range(len(self.limits)):
                if i == 0 and scale_z_vec < self.limits[i]:
                    scale_z_vec = 0
                    break
                if scale_z_vec < self.limits[i]:
                    # The first if-should prevent out of bounds
                    scale_z_vec = self.limits[i - 1]
                    break
            if scale_z_vec not in self.limits:
                # We know that we are given max force
                scale_z_vec = self.limits[-1]

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

