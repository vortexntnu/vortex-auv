#!/usr/bin/env python

import roslib; roslib.load_manifest('waypoint_action_client')
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # from ros-planner/navigation lib
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class WaypointClient():
	def __init__(self):
		rospy.init_node('wp_client', anonymous=True)

		# How big is the square we want the robot to navigate?
		square_size = rospy.get_param("~square_size", 1.0) #meters

		# Create a list to hold the target quaternions (orientations)
		quaternions = list()

		# First define the corner orientations as Euler angles
		euler_angles = (pi/2, pi, 3*pi/2, 0)

		# Then convert the angles to quaterions
		for angle in euler_angles:
			q_angle = quaternion_from_euler(0, 0, angle, axes = 'sxyz')
			q = Quaternion(*q_angle)
			quaternions.append(q)

		# Create a list to hold the waypoint poses
		waypoints = list()

		# Append each of the four waypoints to the list. Each waypoint
		# is a pose consisting of a position and orientation in the map frame
		waypoints.append(Pose(Point(5.0, 2.0, -3.0), quaternions[0]))
		waypoints.append(Pose(Point(15.0, 2.0, -3.0), quaternions[1]))
		waypoints.append(Pose(Point(15.0, -10.0, 0.0), quaternions[2]))
		waypoints.append(Pose(Point(5.0, -10.0, 0.0), quaternions[3]))

		# Initialize the visualization markers for Rviz
		#self.init_markers()


		# set a visualization marker at each waypoint
		#for waypoint in waypoints:
		#	p = Point()
		#	p = waypoint.position
		#	self.markers.points.append(p)


		# Subscrive to the move_base action server
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.on_shutdown(self.shutdown)

		rospy.loginfo("Waiting for move_base action server")

		# Wait 60 seconds for the ation server to become available
		self.move_base.wait_for_server(rospy.Duration(60))

		rospy.loginfo("Connected to move base server")
		rospy.loginfo("Starting navigation test")

		# Initialize a counter to track waypoints
		i = 0

		# Cycle through the four waypoints
		while i < 4 and not rospy.is_shutdown():

			#update the marker display
			#self.marker_pub.publish(self.markers)

			# Initialize the waypoint goal
			goal = MoveBaseGoal()

			# us the map frame to define goals poses
			goal.target_pose.header.frame_id = 'base_link'

			# Set the time stamp to "now"
			goal.target_pose.header.stamp = rospy.Time.now()

			# Set the goal pose to the i-th waypoint
			goal.target_pose.pose = waypoints[i]

			# Start the robot moving toward the goal
			self.move(goal)

			i += 1

	def move(self, goal):
		# Send the goal pose to the MoveBaseAction server
		self.move_base.send_goal(goal)

		# Allow 1 minute to get there
		finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

		# if we don't get there in time, abort goal
		if not finished_within_time:
			self.move_base.cancel_goal()
			rospy.loginfo("Timed out achieving goal")
		else:
			# We made it
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("goal succeeded!")


	def shutdown(self):
		rospy.loginfo("Stopping the robot...")

		# Cancel any active goals
		self.move_base.cancel_goal()
		rospy.sleep(2)


if __name__ == '__main__':
	try:
		node = WaypointClient()
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Test gone wrong")