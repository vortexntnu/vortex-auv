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
from nav_msgs.msg import Odometry

from std_srvs.srv import SetBool

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

		self.sub = rospy.Subscriber('/mission/joystick_data', Joy, self.joystick_data_cb, queue_size=1)
		self.pub = rospy.Publisher('/auv/thruster_manager/input', Wrench, queue_size=1)

		self.joystick_activation_service_server = rospy.Service(
			"/joystick_guidance/activate_joystick_control", SetBool, self.activate_joystick_cb
		)

		self.is_joystick_control_active = false

		self.surge 	= 0
		self.sway 	= 1
		self.heave 	= 2
		self.roll 	= 3
		self.pitch 	= 4
		self.yaw 	= 5

	def joystick_data_cb(self, msg):
		if self.is_joystick_control_active:
			joystick_msg = Wrench()
			joystick_msg.force.x  = msg.axes[self.surge]
			joystick_msg.force.y  = msg.axes[self.sway]
			joystick_msg.force.z  = msg.axes[self.heave]
			joystick_msg.torque.x = msg.axes[self.roll]
			joystick_msg.torque.y = msg.axes[self.pitch]
			joystick_msg.torque.z = msg.axes[self.yaw]

			self.pub.publish(joystick_msg)

	def activate_joystick_cb(self, request):
		self.is_joystick_control_active = request.data

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

