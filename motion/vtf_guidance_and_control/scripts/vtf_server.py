#!/usr/bin/env python

import rospy

# vtf
from guidance_and_control_node import VtfGuidanceAndControlNode, create_wrench_msg

# action message
import actionlib
from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal, VtfPathFollowingResult
from dynamic_reconfigure.server import Server
from vtf_guidance_and_control.cfg import vtf_controllerConfig


class VtfPathFollowing(object):
	"""
	This is the wrapper class for the VtfGuidanceAndControl class. 

	Attributes:
		_result		A vortex_msgs action, true if a goal is set within the
					sphereof acceptance, false if not
	
	Nodes created:
		vtf_server

	Subscribes to:
		/odometry/filtered
	
	"""

	# create messages that are used to send feedback/result
	_result = VtfPathFollowingResult()

	def __init__(self):
		"""
		To initialize the ROS wrapper, the node, subscribers
		 are set up, as well as the action server.
		"""

		rospy.init_node('vtf_server')
		while rospy.get_time() == 0:
			continue

		'''Flag to indicate if a vtf guidance is active or not'''
		self.publish_guidance_data = False

		# parameters
		rate = rospy.get_param("/guidance/vtf/rate", default=20)
		self.ros_rate = rospy.Rate(rate)

		# constructor object
		self.vtf = VtfGuidanceAndControlNode()

		# Action server, see https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/simple_action_server.py
		self.action_server = actionlib.SimpleActionServer(name='vtf_action_server', ActionSpec=VtfPathFollowingAction, auto_start=False)
		self.action_server.register_goal_callback(self.goal_cb)
		self.action_server.start()
		self.vtf_reconfigure_srv = Server(vtf_controllerConfig, self.vtf.vtf_reconfigure)


		rospy.loginfo("vtf guidance initiated")

	def spin(self):
		while not rospy.is_shutdown():
			try:
				if self.publish_guidance_data:
					self.vtf.publish_control_forces()
					self.statusActionGoal()
					self.ros_rate.sleep()
			except rospy.ROSInterruptException:
				pass
			

		
	def statusActionGoal(self):
		"""
		Checks if a preempt request has been set or if the goal was reached. 
		For both cases it publishes a 0 wrench to the thruster manager 
		and sets the action server to preempted or succeeded. 
		"""
		if self.action_server.is_preempt_requested():
			rospy.loginfo("Preempted requested by vtf path client")
			self.publish_guidance_data = False
			msg = create_wrench_msg([0,0,0,0,0,0])
			self.vtf.pub.publish(msg)
			self.action_server.set_preempted()

		# succeeded
		if self.vtf.goal_reached:
			self._result.terminalSector = True
			self.publish_guidance_data = False
			self.vtf.goal_reached = False
			self.action_server.set_succeeded(self._result, text="goal completed")
			msg = create_wrench_msg([0,0,0,0,0,0])
			self.vtf.pub.publish(msg)
			

	def goal_cb(self):
		"""
		The goal callback for the action server.

		Once a goal has been recieved from the client, self.publish_guidance_data is set to True
		"""

		_goal = self.action_server.accept_new_goal()
		rospy.logdebug("vtf_guidance recieved new goal")

		#reset goal reached
		self.vtf.goal_reached = False
		# set goal, the first item will be replaced by current position by the vtf controller so init with dummy:
		self.vtf.waypoints = [[6,6,6]] 
		for wp in _goal.waypoints:
			self.vtf.waypoints.append([wp.x,wp.y, wp.z])
		if self.publish_guidance_data:
			self.vtf.update_path(_goal.forward_speed,_goal.heading, [_goal.heading_point.x,_goal.heading_point.y])
		else:
			self.vtf.new_path_recieved(_goal.forward_speed,_goal.heading, [_goal.heading_point.x,_goal.heading_point.y])
			self.publish_guidance_data = True



if __name__ == '__main__':
	try:
		vtf_path_following = VtfPathFollowing()
		vtf_path_following.spin()

	except rospy.ROSInterruptException:
		pass
