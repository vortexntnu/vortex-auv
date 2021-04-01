#!/usr/bin/env python
# Written by Jae Hyeong Hwang
# Copyright (c) 2021, Vortex NTNU.
# All rights reserved.

import rospy

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy

from std_srvs.srv import SetBool

# to configure joystick environment, please refer to http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

class JoystickGuidanceNode():

	def __init__(self):

		rospy.init_node('joystick_guidance')

		self.sub = rospy.Subscriber('/mission/joystick_data', Joy, self.joystick_data_cb, queue_size=1)
		self.pub = rospy.Publisher('/auv/thruster_manager/input', Wrench, queue_size=1)


		self.joystick_activation_service_server = rospy.Service(
			"/joystick_guidance/activate_joystick_control", SetBool, self.toggle_joystick_cb
		)

		self.publish_guidance_data = False

		self.surge 	= 0
		self.sway 	= 1
		self.heave 	= 2
		self.roll 	= 3
		self.pitch 	= 4
		self.yaw 	= 5

	def joystick_data_cb(self, msg):
		if self.publish_guidance_data:
			joystick_msg = Wrench()
			joystick_msg.force.x  = msg.axes[self.surge]
			joystick_msg.force.y  = msg.axes[self.sway]
			joystick_msg.force.z  = msg.axes[self.heave]
			joystick_msg.torque.x = msg.axes[self.roll]
			joystick_msg.torque.y = msg.axes[self.pitch]
			joystick_msg.torque.z = msg.axes[self.yaw]

			self.pub.publish(joystick_msg)

	def toggle_joystick_cb(self, request):
		self.publish_guidance_data = request.data


if __name__ == '__main__':

	try:
		joystick_guidance = JoystickGuidanceNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

