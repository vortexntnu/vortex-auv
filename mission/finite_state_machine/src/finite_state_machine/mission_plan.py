#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from math import pi
from collections import OrderedDict


def setup_task_environment(self):

	# How deep is the pool we want to patrol?
	self.pool_depth = rospy.get_param('~pool_depth', -1.0) # meters

	# Set the low battery threshold (between 0 and 100)
	self.low_battery_threshold = rospy.get_param('~low_battery_threshold',50)

	# How many times should we execute the the patrol loop?
	self.n_patrols = rospy.get_param('~n_patrols', 2)

	# How long do we have to get to each waypoint? 
	self.nav_timeout = rospy.get_param('~nav_timeout', rospy.Duration(40)) # seconds

	# Initilize the patrol counter
	self.patrol_count = 0

	""" Create a list of target quaternions """

	quaternions = list()

	# Define orientations as Euler angles
	euler_angles = (pi/2, pi, 3*pi/2, 0)

	# Then convert angles to quaternions

	for angle in euler_angles:
		q_angle = quaternion_from_euler(0, 0, angle, 'szyx')
		q = Quaternion(*q_angle)
		quaternions.append(q)


	""" Create a list of target waypoints """ 

	self.waypoints = list()

	# Append each of the waypoints to the list.
	self.waypoints.append(Pose(Point( 0.0, 0.0, 0.7*self.pool_depth), quaternions[0]))
	self.waypoints.append(Pose(Point( 3.0,-2.0, 0.7*self.pool_depth), quaternions[1]))
	self.waypoints.append(Pose(Point( 5.0, 0.0, 0.7*self.pool_depth), quaternions[2]))
	self.waypoints.append(Pose(Point(-2.0,-2.0, 0.0*self.pool_depth), quaternions[3]))

	# Create a mapping of points of interest to waypoint locations

	pool_locations = (('startup', self.waypoints[0]),
                     ('gate', self.waypoints[1]),
                     ('pole', self.waypoints[2]),
                     ('docking', self.waypoints[3]))
	
	# Store the mapping as an ordered dictionary so we can visit the target zones in sequence
	self.pool_locations = OrderedDict(pool_locations)

	# Where is the docking station?
	#self.docking_station_pose = (Pose(Point(1, 1, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))