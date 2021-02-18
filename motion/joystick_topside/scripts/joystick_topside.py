#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Written by Jae Hyeong Hwang and Christopher Strøm
# Copyright (c) 2021, Vortex NTNU.
# All rights reserved.

import rospy

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy

# to configure joystick environment, please refer to http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

class JoystickTopsideNode():

	def __init__(self):

		rospy.init_node('joystick_topside')

		self.sub = rospy.Subscriber('/joystick/joy', Joy, self.callback, queue_size=1)
		self.pub = rospy.Publisher('/joystick/topside_input', Joy, queue_size=1)

		self.surge_scaling = rospy.get_param('/joystick/scaling/surge')
		self.sway_scaling  = rospy.get_param('/joystick/scaling/sway')
		self.heave_scaling = rospy.get_param('/joystick/scaling/heave')
		self.roll_scaling  = rospy.get_param('/joystick/scaling/roll')
		self.pitch_scaling = rospy.get_param('/joystick/scaling/pitch')
		self.yaw_scaling   = rospy.get_param('/joystick/scaling/yaw')


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

		# TODO: Buttons to change control mode, should be a service server between this and guidance

		surge 	= axes['vertical_axis_left_stick'] * self.surge_scaling
		sway 	= axes['horizontal_axis_left_stick'] * self.sway_scaling
		heave 	= (axes['RT'] - axes['LT'])/2 * self.heave_scaling
		roll 	= (buttons['RB'] - buttons['LB']) * self.roll_scaling  
		pitch 	= axes['vertical_axis_right_stick'] * self.pitch_scaling
		yaw 	= axes['horizontal_axis_right_stick'] * self.yaw_scaling

		joystick_msg = Joy()
		joystick_msg.axes = [surge, sway, heave, roll, pitch, yaw]

		self.pub.publish(joystick_msg)


if __name__ == '__main__':

	try:
		joystick_topside = JoystickTopsideNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

