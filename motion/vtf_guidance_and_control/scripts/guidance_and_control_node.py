#!/usr/bin/env python
# Written by Aksel Kristoffersen

import rospy
import numpy as np
import tf
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import WrenchStamped, PoseStamped
from path import Path as Path1
from auv_model import AUVModel
from control_allocation import ControlAllocationSystem
from control_system import DPControlSystem
from auv_simulator import AUVSimulator
from virtual_target import VirtualTarget
from current_estimator import CurrentEstimator
from functions import inside_sphere_of_acceptence, ssa, ned_enu_conversion

class VtfGuidanceAndControlNode:
    def __init__(self):

        rospy.Subscriber('/odometry/filtered', Odometry, self.navigation_callback)# Change Sim/Real

        self.pub = rospy.Publisher('/auv/thruster_manager/input_stamped', WrenchStamped, queue_size=1) # Change Sim/Real 

        self.odom_pub = rospy.Publisher('/odometry/filtered_ned', Odometry, queue_size=1) # Odometry ned publisher 

        self.path_pub = rospy.Publisher('/beluga/guidance_and_control_system/path', Path, queue_size=1) #PAth publisher

        self.br = tf.TransformBroadcaster()
        self.br_eta_r = tf.TransformBroadcaster()
        self.get_pose = False

        '''Initial states'''
        self.eta_d = [0, 0, 0.5, 0, 0, 0]
        self.nu_d = [0, 0, 0, 0, 0, 0]
        self.dot_eta_c = [0, 0, 0, 0, 0, 0]
        self.mode = 'path_following'

        '''Initialize Fossen's equations'''
        m = rospy.get_param("/model_parameters/mass")
        r_g = rospy.get_param("/model_parameters/center_of_mass")
        r_b = rospy.get_param("/model_parameters/center_of_buoyancy")
        inertia = np.array(rospy.get_param("/model_parameters/inertia"))
        volume = rospy.get_param("/model_parameters/volume")
        M_A = np.diag(rospy.get_param("/model_parameters/M_A"))
        D = -np.diag(rospy.get_param("/model_parameters/D"))
        rho = rospy.get_param("/model_parameters/water_density")
        g = 9.81
        self.auv_model = AUVModel(m, r_g, r_b, inertia, volume, M_A, D, rho=rho, g=g)

        '''Initialize control allocation system'''
        thruster_positions = np.array(rospy.get_param("/thruster_parameters/positions"))
        thruster_orientations = np.array(rospy.get_param("/thruster_parameters/orientations"))
        rotor_time_constant = rospy.get_param("/thruster_parameters/first_order_time_constant")
        w = rospy.get_param("/guidance_and_control_parameters/control_forces_weights")
        u_min = rospy.get_param("/guidance_and_control_parameters/control_input_saturation_limits")[0]
        u_max = rospy.get_param("/guidance_and_control_parameters/control_input_saturation_limits")[1]
        self.control_allocation_system = ControlAllocationSystem(thruster_positions, thruster_orientations, rotor_time_constant, u_max, u_min, w)

        '''Initialize DP control system'''
        M = self.auv_model.M
        D = self.auv_model.D
        gvect = self.auv_model.gvect
        omega_b = np.array(rospy.get_param("/guidance_and_control_parameters/control_bandwidth"))
        zeta = np.array(rospy.get_param("/guidance_and_control_parameters/relative_damping_ratio"))
        self.dp_control_system = DPControlSystem(M, D, gvect, omega_b, zeta)
        '''Initialize reference model'''
        u_gain = rospy.get_param("/guidance_and_control_parameters/reference_model_control_input_saturation_limit_gain")
        u_min_simulator = u_min*u_gain
        u_max_simulator = u_max*u_gain
        simulator_control_allocation_system = ControlAllocationSystem(thruster_positions, thruster_orientations, rotor_time_constant, u_max_simulator, u_min_simulator, w)
        omega_b_gain = rospy.get_param("/guidance_and_control_parameters/reference_model_control_bandwidth_gain")
        self.omega_b_simulator = [x*omega_b_gain for x in omega_b]
        zeta = [1 ,1, 1, 1, 1, 1]
        simulator_control_system = DPControlSystem(M, D, gvect, self.omega_b_simulator, zeta)
        absolute_relative_velocity_limit = rospy.get_param("/guidance_and_control_parameters/absolute_relative_velocity_limit")
        self.reference_model = AUVSimulator(self.auv_model, simulator_control_allocation_system, simulator_control_system, absolute_relative_velocity_limit)

        '''Initialize path-following controller'''
        u_gain = rospy.get_param("/guidance_and_control_parameters/virtual_target_control_input_saturation_limit_gain")
        u_min_vt= u_min*u_gain
        u_max_vt = u_max*u_gain
        self.vt_actuator_model = ControlAllocationSystem(thruster_positions, thruster_orientations, rotor_time_constant, u_max_vt, u_min_vt, w)
        self.waypoints = [] 
        self.path_following_controller = None 
        self.heading_mode = 'path_dependent_heading' # Use either 'path_dependent_heading' or 'point_dependent_heading'
        self.dot_s_bounds = rospy.get_param("/guidance_and_control_parameters/virtual_target_along_track_speed_saturation_limits")
        self.heading_point = [0,0]

        '''Virtual target position and speed'''
        self.eta_r = [0,0,0,0,0,0]
        self.dot_s = 0

        '''Sphere of acceptance'''
        self.goal_reached = False

        '''Publish frequency'''
        self.publish_rate = rospy.get_param("/guidance_and_control_parameters/publish_rate")
        self.rate = rospy.Rate(self.publish_rate)

        '''TF listeners'''
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def navigation_callback(self, msg):
        if self.get_pose:
            self.eta, self.nu = ned_enu_conversion(extract_from_pose(msg.pose.pose),extract_from_twist(msg.twist.twist))
            publish_ned(self.odom_pub,self.eta,self.nu)
            self.get_pose = False
        else:
            pass

    def get_state_estimates(self):
        self.get_pose = True
        while self.get_pose:
            continue

    def new_path_recieved(self, speed, heading, heading_point):
        self.get_state_estimates()
        self.reference_model.set_initial_conditions(self.eta, self.nu, rospy.get_time())
        self.waypoints[0] = [self.eta[0],self.eta[1],self.eta[2]] #First waypoint is current location
        path = Path1()
        path.generate_G0_path(self.waypoints)
        omega_b_virtual = rospy.get_param("/guidance_and_control_parameters/virtual_target_controller_bandwidths")
        virtual_control_system = DPControlSystem(self.auv_model.M, self.auv_model.D, self.auv_model.gvect, omega_b_virtual, [1, 1, 1, 1, 1, 1])
        
        self.dot_s_bounds = [-speed,speed]
        self.heading_mode = heading
        self.heading_point = heading_point
        
        self.path_following_controller = VirtualTarget(path, self.auv_model, self.vt_actuator_model, virtual_control_system, self.omega_b_simulator, dot_s_bounds=self.dot_s_bounds)
        
        self.publish_path_once(path) #Publishes path -> probably move to pathplanner node once that is created
    
    def update_path(self, speed, heading, heading_point):
        
        self.waypoints[0] = [self.eta_r[0],self.eta_r[1],self.eta_r[2]] #First waypoint is current location
        path = Path1()
        path.generate_G0_path(self.waypoints)
        
        self.path_following_controller.varpi = 0
        self.path_following_controller.path = path
        
        self.publish_path_once(path)
        
    def publish_control_forces(self):
        self.get_state_estimates()
        # Path following mode
        if self.mode == 'path_following':
            final_wp = self.path_following_controller.path.path[-1](1)
            final_orientation = [0, 0, 0] # Parameter server
            pos_tol = 0.1 # Parameter server
            if inside_sphere_of_acceptence(self.eta[:3], final_wp, pos_tol):
                nu_r = np.zeros(6) 
                eta_r = np.zeros(6)
                eta_r[:3] = final_wp
                eta_r[3:] = final_orientation
                self.goal_reached = True
            else:
                eta_r, nu_r, self.dot_s = self.path_following_controller.generate_reference_trajectories(self.eta_d, self.nu_d, rospy.get_time(), self.heading_mode, point=self.heading_point)

        # Pose hold mode
        elif self.mode == 'pose_hold':
            eta_r = [0, 0, 0, 0, 0, 0] # Insert desired pose here (roll and pitch do not work atm) [x, y, z, roll, pitch, yaw] NED
            nu_r = [0, 0, 0, 0, 0, 0]


        
        # Reference model
        if not self.reference_model.online:
            self.reference_model.set_initial_conditions(self.eta, self.nu, rospy.get_time())
        eta_d, nu_d, dot_nu_d = self.reference_model.generate_trajectory_for_dp(rospy.get_time(), 1, 1/float(self.publish_rate), eta_r, nu_ref=nu_r)
        self.eta_d = eta_d[0]
        self.nu_d = nu_d[0]

        # Control System
        tau_c = self.dp_control_system.pid_regulate(self.eta, self.nu, eta_d[0], nu_d[0], [0,0,0,0,0,0], rospy.get_time(), dot_eta_c=self.dot_eta_c)

        # Save virtual target position
        self.eta_r = eta_r

        # Publish virtual target frame
        p = eta_r[:3]
        eul = eta_r[3:]
        q = quaternion_from_euler(eul[0], eul[1], eul[2])
        self.br.sendTransform((p[0], p[1], p[2]), 
                        (q[0], q[1], q[2], q[3]),
                        rospy.Time.now(),
                        "/virtual_target",
                        "/world_ned")
        
        # Publish reference model frame
        p = eta_d[0][:3]
        eul = eta_d[0][3:]
        q = quaternion_from_euler(eul[0], eul[1], eul[2])
        self.br_eta_r.sendTransform((p[0], p[1], p[2]), 
                        (q[0], q[1], q[2], q[3]),
                        rospy.Time.now(),
                        "/reference_model",
                        "/world_ned")
        
        # Publish control forces
        msg = create_wrenchstamped_msg(tau_c, rospy.get_rostime())
        self.pub.publish(msg)
    
    def publish_path_once(self,path):
        msg = Path()
        msg.header.frame_id = '/world_ned'
        for i in range(len(path.path)):
            for j in list(np.linspace(0, 1, 50)):
                p = path.path[i](j)
                psi = path.chi_p[i](j)
                q = quaternion_from_euler(0, 0, psi)
                pose = PoseStamped()
                pose.header.frame_id = '/world_ned'
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.position.z = p[2]
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                msg.poses.append(pose)
        
        self.path_pub.publish(msg)

        # rate = rospy.Rate(10)
        # ctrl_c = False
        # while not ctrl_c:
        #     print("Echo path!")
        #     connections = path_pub.get_num_connections()
        #     if connections > 0:
        #         path_pub.publish(msg)
        #         ctrl_c = True
        #     else:
        #         rate.sleep()
            

def extract_from_pose(pose):
    quaternions = pose.orientation
    euler_angles = euler_from_quaternion([quaternions.x, quaternions.y, quaternions.z, quaternions.w])
    position = (pose.position.x, pose.position.y, pose.position.z)
    return [position[0], position[1], position[2], euler_angles[0], euler_angles[1], euler_angles[2]]

def extract_from_twist(twist):
    linear = (twist.linear.x, twist.linear.y, twist.linear.z)
    angular = (twist.angular.x, twist.angular.y, twist.angular.z)
    return [linear[0], linear[1], linear[2], angular[0], angular[1], angular[2]]



def create_wrenchstamped_msg(tau, t):
    msg = WrenchStamped()
    msg.header.stamp = t
    msg.header.frame_id = "beluga/base_link_ned"
    msg.wrench.force.x = tau[0]
    msg.wrench.force.y = tau[1]
    msg.wrench.force.z = tau[2]
    msg.wrench.torque.x = tau[3]
    msg.wrench.torque.y = tau[4]
    msg.wrench.torque.z = tau[5]
    return msg

def publish_ned(pub, eta, nu):
    odom = Odometry()

    orien = quaternion_from_euler(eta[3],eta[4],eta[5])

    odom.pose.pose.position.x = eta[0]
    odom.pose.pose.position.y = eta[1]
    odom.pose.pose.position.z = eta[2]
    odom.pose.pose.orientation.x = orien[0]
    odom.pose.pose.orientation.y = orien[1]
    odom.pose.pose.orientation.z = orien[2]
    odom.pose.pose.orientation.w = orien[3]

    odom.twist.twist.linear.x = nu[0]
    odom.twist.twist.linear.y = nu[1]
    odom.twist.twist.linear.z = nu[2]
    odom.twist.twist.angular.x = nu[3]
    odom.twist.twist.angular.y = nu[4]
    odom.twist.twist.angular.z = nu[5]

    odom.header.frame_id = '/world_ned'

    pub.publish(odom)



