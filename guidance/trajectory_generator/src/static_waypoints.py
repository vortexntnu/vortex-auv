#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2019 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
import numpy as np
from vortex_msgs.msg import PropulsionCommand
from geometry_msgs.msg import Pose


class Waypoints:

	def referenceModel(self,last, next, time):
		
		a = 0.3
		self.new_arr = [last.position.x, last.position.y, last.position.z]
		next_arr = [next.position.x, next.position.y, next.position.z]
					

		for i in range (len(next_arr)):
			
			# calculate current distance
			if (next_arr[i] < 0 and self.new_arr[i] > 0) or (next_arr[i] > 0 and self.new_arr[i] < 0):
				distance = np.absolute(self.new_arr[i])+np.absolute(next_arr[i])				
			else:
				distance = np.absolute(np.absolute(self.new_arr[i])-np.absolute(next_arr[i]))
				
			
			# next waypoint
			if next_arr[i] < self.new_arr[i]:
				self.new_arr[i] = self.new_arr[i] - distance*(1-np.exp(-a*time))
			else:
				self.new_arr[i] = self.new_arr[i] + distance*(1-np.exp(-a*time))

		return self.new_arr

		
	
	def publishWaypoint(self):

		# init Position
		last = Pose()
		last.position.x = 5.0
		last.position.y = -10.0
		last.position.z = 0.0
		
		# set position
		next = Pose()
		next.position.x = 14.0
		next.position.y = 1.0
		next.position.z = -5.0

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
		dt = 0.05
		time = 0
		#smooth_wp = Pose()

		while not rospy.is_shutdown():
			
			#wp_arr = self.referenceModel(last, next, time)
			#print ("error : ", wp_arr) 
			## update waypoint
			#smooth_wp.position.x = wp_arr[0]
			#smooth_wp.position.y = wp_arr[1]
			#smooth_wp.position.z = wp_arr[2]
			
			# set waypoint
			self.pub_wp.publish(next)
			self.pub_mode.publish(mode)
			self.rate.sleep()
			time += dt


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
		self.pub_mode = rospy.Publisher('/manta/mode', PropulsionCommand,	 queue_size=1)

		# spin
		self.publishWaypoint()



# ROS spin
if __name__ == '__main__':
	try:
		node = Waypoints()
	except rospy.ROSInterruptException:
		print('caught exeption')
	print('exiting')	
