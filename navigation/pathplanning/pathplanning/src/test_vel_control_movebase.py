#!/usr/bin/env python

import rospy
import numpy as np

from smach import State, StateMachine, Sequence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer

# action message
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Wrench
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist


class PIDRegulator:
	""" A very basic 1D PID regulator """
	def __init__(self, p, i, d, sat):
		self.p = p
		self.i = i
		self.d = d
		self.sat = sat

		self.integral = 0
		self.prev_err = 0
		self.prev_t = -1

	def __str__(self):
		msg = 'PID controller:'
		msg += '\n\tp=%f' % self.p
		msg += '\n\ti=%f' % self.i
		msg += '\n\td=%f' % self.d
		msg += '\n\tsat=%f' % self.sat
		return msg

	def regulate(self, err, t):

		derr_dt = 0.0
		dt = t - self.prev_t
		if self.prev_t > 0.0 and dt > 0.0:
			derr_dt = (err - self.prev_err)/dt
			self.integral += 0.5*(err + self.prev_err)*dt

		u = self.p*err + self.d*derr_dt + self.i*self.integral

		self.prev_err = err
		self.prev_t = t

		if (np.linalg.norm(u) > self.sat):
			# controller is in saturation: limit output, reset integral
			u = self.sat*u/np.linalg.norm(u)
			self.integral = 0.0

		return u

class TaskManager():

    def __init__(self):


        rospy.init_node('test_contr_state_machine', anonymous=False)

        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.path = []

        self.vehicle_vel = Odometry()
        self.sub_vel = rospy.Subscriber('/odometry/filtered', Odometry, self.velocityCallback, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmdvelCallback, queue_size=1)
        self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)


        print("Rospy spin")
        rospy.spin()
        print("Finished TaskManager")

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)

    def cmdvelCallback(self, msg):
        print(msg)
        cmd_vel_x = msg.linear.x
        cmd_vel_y = msg.linear.y
        cmd_z_twist = msg.angular.z

        cmdWrench = Wrench()
        PID_vel_x = PIDRegulator(45, 20, 0, np.Inf)
        PID_vel_y = PIDRegulator(30, 5, 0, np.Inf)
        PID_angvel_z = PIDRegulator(20, 10, 0, np.Inf)

        vel_x_error = cmd_vel_x - self.vel_x
        vel_y_error = cmd_vel_y - self.vel_y
        angvel_z_error = cmd_z_twist - self.angvel_z
        print("vel_x_error:", np.round(vel_x_error, 1))
        print("vel_y_error:", np.round(vel_y_error, 1))
        print("angvel_z_error:", np.round(angvel_z_error, 1))
        
        cmdWrench.force.x = PID_vel_x.regulate(vel_x_error,self.time)
        cmdWrench.force.y = PID_vel_y.regulate(vel_y_error, self.time)
        cmdWrench.torque.z = PID_angvel_z.regulate(angvel_z_error, self.time)


        print("Forcex: ", cmdWrench.force.x, " Forcey: ", cmdWrench.force.y, " Torquez: ", cmdWrench.torque.z)

        self.pub_thrust.publish(cmdWrench)

    
    def velocityCallback(self, msg):
        self.vehicle_vel = msg
        self.time = msg.header.stamp.to_sec()

        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.angvel_z = msg.twist.twist.angular.z

if __name__ == '__main__':
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")