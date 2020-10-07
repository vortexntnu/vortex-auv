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
from visualization_msgs.msg import Marker, MarkerArray
#from autopilot.autopilot import AutopilotBackstepping, AutopilotPID



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
        
        rospy.sleep(3)

        print("Started node test_controller_state_machine")

        rospy.init_node('test_contr_state_machine', anonymous=False)

        self.x = 0
        self.y = 0
        self.z = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.path = []

        self.vehicle_odom = Odometry()
        self.sub_pose = rospy.Subscriber('/odometry/filtered', Odometry, self.positionCallback, queue_size=1)
        self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)
        self.marker_pub = rospy.Publisher('/pathplanning/closest_pt', Marker, queue_size=1)
        self.get_plan = rospy.ServiceProxy('/move_base_node/make_plan', GetPlan)

        rospy.sleep(3)

        goal = PoseStamped()
        goal.header.frame_id = "manta/odom"
        goal.pose.position.x = 10
        goal.pose.position.y = 10
        goal.pose.position.z = 0
        self.makePlanToGoal(goal)

        self.switched_plan = False
        self.start_time = rospy.get_time()

        P_yaw = 25
        I_yaw = 0
        D_yaw = 10
        sat_yaw = 100
        self.PID_yaw = PIDRegulator(P_yaw,I_yaw,D_yaw,sat_yaw)  # p, i, d, sat

        P_side = 20
        I_side = 0
        D_side = 10
        sat_side = 100
        self.PID_side = PIDRegulator(P_side, I_side, D_side, sat_side)
        

        print("Rospy spin")
        rospy.spin()
        print("Finished TaskManager")
    
    def makeNewPlan(self):
        if self.x > 2 and not self.switched_plan:
            self.switched_plan = True
            goal = PoseStamped()
            goal.header.frame_id = "manta/odom"
            goal.pose.position.x = 0
            goal.pose.position.y = -9
            goal.pose.position.z = 0
            self.makePlanToGoal(goal)
            


        

    def shutdown(self):
        rospy.loginfo("stopping the AUV...")
        rospy.sleep(10)

    def findVectorSide(self, path_pos):
        print("h")

    def controller(self):

        index,closest_pt = self.findClosestPointIndex()

        yaw = self.getYawFrom2Pos(self.path[index].pose.position, self.path[index+1].pose.position)
        
        testWrench = Wrench()
        force_x = 10 
        testWrench.force.x = force_x
        err_yaw = yaw - self.yaw
        if err_yaw > 3.14:
            err_yaw = err_yaw - 6.28

        if err_yaw < -3.14:
            err_yaw = err_yaw + 6.28

        vec_toward_path = np.array([closest_pt.x - self.x, closest_pt.y - self.y])
        rot_mat = np.array([[np.cos(self.yaw), -np.sin(self.yaw)],[np.sin(self.yaw), np.cos(self.yaw)]])
        rot_mat = rot_mat.transpose()

        dir_manta_frame = np.dot(rot_mat, vec_toward_path)

        err_side = dir_manta_frame[1]


        testWrench.torque.z = self.PID_yaw.regulate(err_yaw, rospy.get_time())
        force_y = self.PID_side.regulate(err_side, rospy.get_time())
        testWrench.force.y = force_y

        force_x = max(force_x - 10*abs(err_side) - 10*abs(err_yaw), 3)
        
        self.pub_thrust.publish(testWrench)

        print_info = True
        if print_info:
            print("contr")
            print("index: ", index, " closest_point:", closest_pt)
            print("Self yaw: ", self.yaw, "  Point Yaw: ", yaw)
            print("Error yaw: ", err_yaw)
            print("Error side: ", err_side, "   x_component: ", dir_manta_frame[0])
            print("Force x: ", force_x, " Force y: ", force_y, "Torque z: ", testWrench.torque.z )






    def getYawFrom2Pos(self, pos1, pos2):
        if (pos2.x - pos1.x ) == 0:
            return 0

        return np.arctan2((pos2.y - pos1.y),(pos2.x - pos1.x))


    
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

        self.makeNewPlan()

        if len(self.path) > 0:
            self.controller()

    def goForward(self):
        testWrench = Wrench()
        testWrench.force.x = 30
        #testWrench.torque.z = 100
        
    def findClosestPointIndex(self):
        distArray = []
        for pose in self.path:
            distArray.append(abs(self.x-pose.pose.position.x)**2 + abs(self.y - pose.pose.position.y)**2)

        minIndex = np.argmin(distArray)
        closest_pt = self.path[minIndex].pose.position

        marker = Marker()
        marker.pose.position.x = closest_pt.x
        marker.pose.position.y = closest_pt.y
        marker.header.frame_id = "manta/odom"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

        return minIndex, closest_pt
        

    def makePlanToGoal(self, goal):

        print("waiting for service")
        rospy.wait_for_service('move_base_node/make_plan')
        print("finished waiting for service")
        

        start = PoseStamped()
        
        start.header.frame_id = "manta/odom"
        start.pose.position.x = self.x
        start.pose.position.y = self.y
        start.pose.position.z = self.z


        

        tolerance = 0

        print("Start:")
        print(start)
        print("Goal: ")
        print(goal)


        plan_response = self.get_plan(start = start, goal = goal, tolerance = tolerance)
        print("Plan response type: ")
        print(type(plan_response))
    
        poses_arr = plan_response.plan.poses

        self.path = poses_arr

        print("Lengde: array ", len(poses_arr))
        #   print(poses_arr)


if __name__ == '__main__':
	try:
		TaskManager()
	except rospy.ROSInterruptException:
		rospy.loginfo("Pathplanning state machine has been finished")