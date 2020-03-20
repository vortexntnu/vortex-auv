#!/usr/bin/env python
import rospy
import numpy as np
import math
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Wrench
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from autopilot.autopilot import AutopilotBackstepping, AutopilotPID
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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


class InspectPoint:
    def __init__(self):
        print("inspect unknown point")
        rospy.init_node('inspect_unknown_point')

        # init variables
        self.run_controller = False

        self.x = 0
        self.y = 0
        self.z = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.centre_of_rot = Point()
        self.desired_radius = 10

        self.desired_depth = -0.5

        # subscribers
        self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)

        # publishers
        self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)

        # action server setup
        self.action_server = actionlib.SimpleActionServer(name='inspect_point_srv', ActionSpec=LosPathFollowingAction, auto_start=False)
        self.action_server.register_goal_callback(self.goalCB)
        self.action_server.start()

        # pid regulators
        P_dist = 20
        I_dist = 0
        D_dist = 10
        sat_dist = 5
        self.PID_dist = PIDRegulator(P_dist, I_dist, D_dist, sat_dist)

        P_angle = 20
        I_angle = 10
        D_angle = 10
        sat_angle = 5
        self.PID_angle = PIDRegulator(P_angle, I_angle, D_angle, sat_angle)

        self.PID = AutopilotPID()

    def fixHeadingWrapping(self, err_yaw):

        if err_yaw > math.pi:
            err_yaw = err_yaw - 2*math.pi

        if err_yaw < -math.pi:
            err_yaw = err_yaw + 2*math.pi

        return err_yaw

        
    def getVectorFromMantaToCentre(self):
        vec_to_mid = np.array([0,0])

        vec_to_mid[0] = self.centre_of_rot.x - self.x
        vec_to_mid[1] = self.centre_of_rot.y - self.y

        return vec_to_mid

    def distanceToMid(self):
        return np.sqrt(abs(self.x-self.centre_of_rot.x)**2 + abs(self.y - self.centre_of_rot.y)**2)

    def getYawFrom2Pos(self, pos1, pos2):
        #if (pos2.x - pos1.x ) == 0:
        #    return 0.0

        return np.arctan2((pos2.y - pos1.y),(pos2.x - pos1.x))


    def controller(self):
        
        # distance control
        dist = self.distanceToMid()

        err_dist = dist - self.desired_radius

        pid_output = self.PID_dist.regulate(err_dist, rospy.get_time())

        vec_to_mid_glob_frame = self.getVectorFromMantaToCentre()

        rot_mat = np.array([[np.cos(self.yaw), -np.sin(self.yaw)],[np.sin(self.yaw), np.cos(self.yaw)]])
        rot_mat = rot_mat.transpose()

        vec_to_mid_manta_frame = np.dot(rot_mat,vec_to_mid_glob_frame)


        force_vec = vec_to_mid_manta_frame * pid_output

        force_x = force_vec[0]
        force_y = force_vec[1]

        


        # angle control
        manta_pos = Point()
        manta_pos.x = self.x
        manta_pos.y = self.y
        desired_angle = self.getYawFrom2Pos(manta_pos, self.centre_of_rot)

        err_yaw = desired_angle - self.yaw

        err_yaw = self.fixHeadingWrapping(err_yaw)

        torque_z = self.PID_angle.regulate(err_yaw, rospy.get_time())

        # sideways force to rotate around point
        # dont go sideways unless facing towards point

        max_side_force = 5

        sideway_force = max(0, max_side_force - 5*err_yaw)

        force_y = force_y + sideway_force
            
        # depth control
        tau_depth_hold = self.PID.depthController(self.desired_depth, self.z, rospy.get_time())

        # make wrench and publish
        wrench = Wrench()
        wrench.force.x = force_x
        wrench.force.y = force_y
        wrench.torque.z = torque_z
        wrench.force.z = tau_depth_hold

        self.pub_thrust.publish(wrench)

        PRINT_INFO = True
        if PRINT_INFO: 
            print("Force_x: ", force_x, "   Force_y: ", force_y, "  torque_z: ", torque_z)
            print("Error distance: ", err_dist, "   Error yaw: ", err_yaw)

    
    def goalCB(self):
        print("got goal")
        self.run_controller = True
        goal = self.action_server.accept_new_goal()

        self.centre_of_rot.x = goal.next_waypoint.x
        self.centre_of_rot.y =  goal.next_waypoint.y
        self.desired_radius = goal.sphereOfAcceptance

        print("cor x: ", self.centre_of_rot.x)
        print("cor y: ", self.centre_of_rot.y)
        print("des rad:", self.desired_radius)



    def distanceBetweenPoseAndSelf(self, pose):
        return np.sqrt(self.x-pose.position.x)**2 + abs(self.y - pose.position.y)**2


    def positionCallback(self, msg):
        self.vehicle_odom = msg
        self.time = msg.header.stamp.to_sec()
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll,pitch,yaw) = euler_from_quaternion(orientation_list)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        if self.run_controller:
            self.controller()




if __name__ == '__main__':
	try:
		insepctObj = InspectPoint()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
