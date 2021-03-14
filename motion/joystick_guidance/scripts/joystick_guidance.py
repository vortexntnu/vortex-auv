#!/usr/bin/env python
# Written by Jae Hyeong Hwang and O. Solbo
# Copyright (c) 2021, Vortex NTNU.
# All rights reserved.


import rospy

from geometry_msgs.msg import Wrench, Pose
from sensor_msgs.msg import Joy
from math import sqrt
from std_msgs.msg import Bool
from pyquaternion import Quaternion

# to configure joystick environment, please refer to http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick


def convert_to_global(local_point, pose):
	"""
	Returns the point converted into the global framework when
	taking the pose into account
	"""
	# Initializing the global point
	global_point = local_point

	# Calculating the quaternion and rotating the vector
	quaternion = Quaternion([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
	rotated_vector = quaternion.rotate(local_point)

	# Linar scaling the global point with position and orientation
	global_point[0] += (pose.position.x + rotated_vector[0])
	global_point[1] += (pose.position.y + rotated_vector[1])
	global_point[2] += (pose.position.z + rotated_vector[2])

	return global_point


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
		if abs(value) < ref_list[i]:
			temp_value = decimal_list[i]
			break

	# We allow maximum thrust when it is given below a certain threshold
	if abs(value) >= ref_list[-1] - deadzone:
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

		self.sub_joystick_data = rospy.Subscriber('/guidance/joystick_data', Joy, self.joystick_data_cb, queue_size=1)
		self.sub_odometry_filtered = rospy.Subscriber('/odometry/filtered', Pose, self.odometry_cb, queue_size=1)
		# self.pub = rospy.Publisher('/auv/thruster_manager/input', Wrench, queue_size=1) # Uncomment to run the thrusters directly
		self.pub_joy = rospy.Publisher('/guidance/joystick_reference', Pose, queue_size=1)
		self.pub_state = rospy.Publisher('/guidance/joystick_state', Bool, queue_size=1)

		# Joystick-state indicates if the UUV should be in ROV- or AUV-mode. Controlled by 'start'
		# False <=> AUV
		# True 	<=> ROV
		self.start_pressed = False
		self.start_button_idx = 9

		# Bool used to prevent invalid operations in case odometry is not pushed to
		self.odometry_published = False

		self.surge 	= 0
		self.sway 	= 1
		self.heave 	= 2
		self.roll 	= 3 
		self.pitch 	= 4 
		self.yaw 	= 5 

		self.min_point_range = 0.2                          # Minimum allowed distance from the UUV to the point
		self.max_point_range = 1.0                          # Maximum allowed distance from the UUV to the calculated point
		self.num_ranges 	 = 5                            # Number of ranges the input is scaled by
		self.deadzone 		 = 1.0	   						# Deadzone on upper input

		# Required input-values to generate valid signals
		limits = []
		self.max_value = 10.0
		for i in range(1, self.num_ranges + 1):         
			limits.append((self.max_value / self.num_ranges) * i)
		self.limits = limits


		# Calculated input in decimal
		decimal_list = []
		for i in range(0, self.num_ranges + 1):
			decimal_list.append((self.max_point_range / self.num_ranges) * i)
		self.decimal_list = decimal_list


	def odometry_cb(self, msg):
		self.odometry_published = True
		self.uuv_pose = msg


	def joystick_data_cb(self, msg):
		
		# Calculating thrust and publish to thrusters 
		joystick_msg = Wrench()
		joystick_msg.force.x  = msg.axes[self.surge]
		joystick_msg.force.y  = msg.axes[self.sway]
		joystick_msg.force.z  = msg.axes[self.heave]
		joystick_msg.torque.x = msg.axes[self.roll]
		joystick_msg.torque.y = msg.axes[self.pitch]
		joystick_msg.torque.z = msg.axes[self.yaw]

		"""
		# Uncomment if the thrust-vector should be given directly to the thrusters
		self.pub.publish(joystick_msg)
		"""

		start_command = msg.buttons[self.start_idx]
		if start_command == 1:
			self.start_pressed = not self.start_pressed

			bool_msg = Bool()
			mool_msg.data = self.start_pressed
			self.pub_state.publish(bool_msg)

			# Sleep for 0.25 seconds to prevent the system from constantly switching between ROV and AUV
			rospy.Duration(0.25).sleep()


		# Calculating point and publishing to DP-controller
		point = [0, 0, 0]
		if self.odometry_published:
			point = self.calculate_global_point(joystick_msg)

		pose_msg = Pose()
		pose_msg.position.x   	= point[0]
		pose_msg.position.y   	= point[1]
		pose_msg.position.z   	= point[2]
		pose_msg.orientation.x 	= 0
		pose_msg.orientation.y 	= 0
		pose_msg.orientation.z 	= 0
		pose_msg.orientation.w 	= 0

		self.pub_joy.publish(pose_msg)

	def calculate_global_point(self, joystick_msg):
		"""
		Calculates a point in the local frame based on the given thrust-vectors 
		from joystick 

		If required it must be converted into the global reference-frame
		"""

		# Scaling the force to get a linearized model
		scaled_force_x, scaled_force_y, scaled_force_z = self.scale_force_vectors(joystick_msg)

		local_calculated_point = [scaled_force_x, scaled_force_y, scaled_force_z]

		# Calculating the length of the vectors
		vector_length_square = 0
		for i in range(len(local_calculated_point)):
			vector_length_square += pow(local_calculated_point[i], 2)

		# Normalizing if exceeding the max_point_range
		if vector_length_square >= pow(self.max_point_range, 2):
			# Over the set limit. Normalizing
			vector_length = sqrt(vector_length_square)
			local_calculated_point = [val / vector_length for val in local_calculated_point]

		# Converting to the local point
		global_calculated_point = convert_to_global(local_calculated_point, self.uuv_pose)

		return global_calculated_point


	def scale_force_vectors(self, joystick_msg):
		"""
		Scales the thrust-vectors to
		"""

		# Recovering the given forces
		# Scaling force_z with 2 since heave is /2 in guidance_interface.py. This is to 
		# simplify the mathematics/code in this program
		force_x = joystick_msg.force.x
		force_y = joystick_msg.force.y
		force_z = joystick_msg.force.z * 2			

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
		#rospy.logerr("An error occured during startup")
		pass

