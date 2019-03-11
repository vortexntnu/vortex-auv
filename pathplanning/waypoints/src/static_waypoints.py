#!/usr/bin/env python

import rospy
import numpy as np
from vortex_msgs.msg import PropulsionCommand
from geometry_msgs.msg import Pose


class Waypoints:
	
	def publishWaypoint(self):

		# Set Position
		wp = Pose()
		wp.position.x = 14
		wp.position.y = 1
		wp.position.z = -5

		# Set mode
		mode = PropulsionCommand()
		mode.control_mode = [
			(False),
			(False),
			(False),
			(False),
			(False),
			(False),
		]

		# Get time stamp
		mode.header.stamp = rospy.get_rostime()

		while not rospy.is_shutdown():

			self.pub_wp.publish(wp)
			self.pub_mode.publish(mode)
			self.rate.sleep()


	#constructor of the class
	#self refers to the instance of the object (like "this" in C++)
	#__init__ gets called when memory of the object is allocated
	def __init__(self): 	
		
		# Initialize the node and name it
		rospy.init_node('waypointPublisher')

				# spin rate
		self.rate = rospy.Rate(20) #20hz 

		# ROS infrastructure
		self.pub_wp = rospy.Publisher('/manta/waypoints', Pose, queue_size=1)
		self.pub_mode = rospy.Publisher('/manta/mode', PropulsionCommand, queue_size=1)

		# spin
		self.publishWaypoint()



# ROS spin
if __name__ == '__main__':
	try:
		node = Waypoints()
	except rospy.ROSInterruptException:
		print('caught exeption')
	print('exiting')
