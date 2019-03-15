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
from tf.transformations import quaternion_from_euler

		#  	"atan2"
		#            y 90
		#	  180	 |     0         ^
		# 		 <---*----> x   rot+ |
		#	 -180	 |     0
		#           -90
		#

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
						circleOfAcceptance = wp['R']
						self.waypoints.append((x,y,z,circleOfAcceptance))
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
		self.rate = rospy.Rate(20) #20hz 
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


	def distance(self, pos, wp):
		return 	np.sqrt((wp[0]-pos[0])**2 +
					    (wp[1]-pos[1])**2 +
					    (wp[2]-pos[2])**2)

	def circle_of_acceptance(self, distance, R):
		return distance < R



	def target_heading(self, pos, wp):

		dx = wp[0] - pos[0]
		dy = wp[1] - pos[1]
		return np.arctan2(dy,dx)


	def positionCallback(self):

		# get current position
		pos = [20, -12, -3]

		# get current setpoint
		wps = self.guidance.waypoints
		wp = wps[0]
		R = wp[3]
		distance = self.distance(pos,wp)

		# if auv is within circle of acceptance
		# get new waypoint
		if (self.circle_of_acceptance(distance, R)):
			self.guidance.delete_waypoint(wps)
			wp = self.wps[0]


		# calc heading in euler and transform to quaternions
		heading = self.target_heading(pos,wp)
		quat = quaternion_from_euler(0.0,0.0,heading)

		while not rospy.is_shutdown():
			# setpoint
			self.wp_msg.position.x = wp[0]
			self.wp_msg.position.y = wp[1]
			self.wp_msg.position.z = wp[2]
			self.wp_msg.orientation.x = quat[0]
			self.wp_msg.orientation.y = quat[1]
			self.wp_msg.orientation.z = quat[2]
			self.wp_msg.orientation.w = quat[3]
			self.pub_waypoint.publish(self.wp_msg)
			self.rate.sleep()		
		#for wp in wps:
		#	print('Point: ', wp)

		#self.guidance.delete_waypoint(wps)
		#print 'delete'
		#for wp in wps:
		#	print('Point', wp)

		

# ROS spin
if __name__ == '__main__':

	try:
		node = WaypointTracking()
		rospy.spin()
	except rospy.ROSInterruptException:
		print('caught exeption')
	print('exiting')

