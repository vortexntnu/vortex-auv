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

		self.sub = rospy.Subscriber('/joystick/joy', Joy, self.callback, queue_size=1)

		self.pub = rospy.Publisher('/auv/thruster_manager/input', Wrench, queue_size=1)

		self.surge_scaling = rospy.get_param('/joystick/scaling/surge', 1)
		self.sway_scaling  = rospy.get_param('/joystick/scaling/sway', 1)
		self.heave_scaling = rospy.get_param('/joystick/scaling/heave', 1)
		self.roll_scaling  = rospy.get_param('/joystick/scaling/roll', 1)
		self.pitch_scaling = rospy.get_param('/joystick/scaling/pitch', 1)
		self.yaw_scaling   = rospy.get_param('/joystick/scaling/yaw', 1)


		# Name buttons and axes based on index from joy-node
		self.buttons_map = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'back',
							'start', 'power', 'stick_button_left',
							'stick_button_right']

		self.axes_map = ['horizontal_axis_left_stick',
						'vertical_axis_left_stick', 'LT',
						'horizontal_axis_right_stick',
						'vertical_axis_right_stick', 'RT',
						'dpad_horizontal', 'dpad_vertical']


	def callback(self, msg):

		buttons = {}
		axes = {}

		for i in range(len(msg.buttons)):
			buttons[self.buttons_map[i]] = msg.buttons[i]

		for j in range(len(msg.axes)):
			axes[self.axes_map[j]] = msg.axes[j]

		surge 	= axes['vertical_axis_left_stick'] * self.surge_scaling
		sway 	= axes['horizontal_axis_left_stick'] * self.sway_scaling
		heave 	= (axes['RT'] - axes['LT'])/2 * self.heave_scaling
		roll 	= (buttons['RB'] - buttons['LB']) * self.roll_scaling  
		pitch 	= axes['vertical_axis_right_stick'] * self.pitch_scaling
		yaw 	= axes['horizontal_axis_right_stick'] * self.yaw_scaling

		joystick_msg = Wrench()
		joystick_msg.force.x  = surge
		joystick_msg.force.y  = sway
		joystick_msg.force.z  = heave
		joystick_msg.torque.x = roll  
		joystick_msg.torque.y = pitch
		joystick_msg.torque.z = yaw

		self.pub.publish(joystick_msg)


if __name__ == '__main__':

	try:
		joystick_guidance = JoystickGuidanceNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

