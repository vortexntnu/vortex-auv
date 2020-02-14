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
	self.transit_speed = rospy.get_param('~transit_speed', 0.3)

	# Search area size
	self.los_sphere_of_acceptance = rospy.get_param('~search_area_size', 0.7)
	self.search_depth = rospy.get_param('~search_depth', 0.5*self.pool_depth)
	self.search_speed = rospy.get_param('~search_speed',0.2)

	# Set the low battery threshold (between 0 and 100)
	self.low_battery_threshold = rospy.get_param('~low_battery_threshold',50)

	# How many times should we execute the the patrol loop?
	self.n_patrols = rospy.get_param('~n_patrols', 2)

	# How long do we have to get to each waypoint? 
	self.nav_timeout = rospy.get_param('~nav_timeout', rospy.Duration(60)) # seconds

	# Initilize the patrol counter
	self.patrol_count = 0

	""" Create a list of target quaternions """

	pool_locations = (
		('corner_1', make_waypoint(0, 0)),
        ('corner_2', make_waypoint(15,0)),
        ('corner_3', make_waypoint(4, 4)),
		('corner_4', make_waypoint(0, 4))
	)
	
	# Store the mapping as an ordered dictionary so we can visit the target zones in sequence
	self.pool_locations = OrderedDict(pool_locations)

	# Where is the docking station?
	#self.docking_station_pose = (Pose(Point(1, 1, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))


def make_waypoint(x, y, z=-0.5, yaw_euler=0):

	yaw_quat = Quaternion(*quaternion_from_euler(0, 0, yaw_euler, 'sxyz'))
	waypoint = Pose(Point( x, y, z), yaw_quat)

	return waypoint

