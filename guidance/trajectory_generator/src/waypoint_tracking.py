#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2019 Manta AUV, Vortex NTNU.
# All rights reserved.

import os
import rospy
import numpy as np
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

class PrepareWaypoints:

	def __init__(self, inertial_frame_id='world'):
		assert inertial_frame_id in ['world', 'world_enu']
		self._inertial_frame_id = inertial_frame_id
		self.waypoints = []

	def read_from_file(self, filename):
		if not os.path.isfile(filename):
			print 'Invalid waypoint filename, file', filename
			return False
		try:
			with open(filename, 'r') as wp_file:
				wps = yaml.load(wp_file)

				# check if .yaml has a list
				if isinstance(wps['waypoints'], list):
					
					# pick every waypoint an insert into array
					for wp in wps['waypoints']:
						x = wp['point'][0]
						y = wp['point'][1]
						z = wp['point'][2]
						heading = wp['heading']
						self.waypoints.append((x,y,z,heading))
					#print(self.waypoints)

		except Exception, e:
			print'Error when loading the waypoint file'
			print str(e)
		   	return False
		return True
		
	def delete_waypoint(self,waypoints):
		# pop first waypoint in list
		waypoints.pop(0)

class WaypointTracking(object):
	
	# Contstructor of the class
	# self refers to the instance of the object (like "this" in C++)
	# __init__ gets called when memory of the object is allocated
	def __init__(self):
		# initialize node name
		rospy.init_node('waypoint_tracking_node')

		# get file
		if rospy.is_shutdown():
		    rospy.logerr('ROS master not running!')
		    sys.exit(-1)
		    rospy.logerr('Found waypoint file!')
		if rospy.has_param('~filename'):

		    filename = rospy.get_param('~filename')
		else:
		    raise rospy.ROSException('No filename found')

		# Subscriber
		self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)
		
		# Publisher
		self.pub_waypoint = rospy.Publisher('/manta/waypoints', Pose, queue_size=1)

		# Initialize publishing message
		self.wp_msg = Pose()




		# Start tracking by reading waypoints from file
		self.guidance = PrepareWaypoints()
		self.guidance.read_from_file(filename)
		self.positionCallback()

		
	def positionCallback(self):

		wps = self.guidance.waypoints

		for wp in wps:
			print('Point: ', wp)

		self.guidance.delete_waypoint(wps)
		print 'delete'
		for wp in wps:
			print('Point', wp)

		

# ROS spin
if __name__ == '__main__':

	try:
		node = WaypointTracking()
		rospy.spin()
	except rospy.ROSInterruptException:
		print('caught exeption')
	print('exiting')

