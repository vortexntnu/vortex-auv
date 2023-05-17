import rospy
import rosnode
import actionlib
import subprocess

from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix
import numpy as np

from geometry_msgs.msg import Wrench, Pose
from vortex_msgs.msg import dpAction, dpGoal, dpResult
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse 

from libjoystick.JoystickControlModes import *


class ControlModeHandling:

    def __init__(self):
        self.control_mode = JoystickControlModes(0)
        self.odom_pose = Pose()
        self.prev_time = 0.0
        
        # DP client
        self.dp_client = actionlib.SimpleActionClient("/DpAction", # Server name
                                                      dpAction)
        self.refmodel_client = rospy.ServiceProxy('/dp_reference_model/enable_x_ref_filter', SetBool)
        self.refmodel_req = SetBoolRequest()
        self.refmodel_req.data = True

    def control_mode_change(self, buttons, wrench_publisher_handle):
        pressed = -1

        if buttons["stick_button_left"] and buttons[
                "stick_button_right"] and buttons["RB"] and buttons["LB"]:
            pressed = JoystickControlModes.KILLSWITCH.value
            self.control_mode = pressed
            ControlModeHandling.killswitch(wrench_publisher_handle)

        if buttons["start"]:
            pressed = JoystickControlModes.EMERGENCY_STOP.value
            self.control_mode = pressed
            ControlModeHandling.emergency_stop(wrench_publisher_handle)

        elif buttons["A"]:
            pressed = JoystickControlModes.OPEN_LOOP.value
            self.control_mode = pressed
            if not self.refmodel_req.data:
                self.refmodel_req.data = True
                res = self.refmodel_client(self.refmodel_req.data)
            self.open_loop()

        elif buttons["B"]:
            pressed = JoystickControlModes.POSE3_HOLD.value
            self.control_mode = pressed
            if not self.refmodel_req.data:
                self.refmodel_req.data = True
                res = self.refmodel_client(self.refmodel_req.data)
            self.pose3_hold()

        elif buttons["X"]:
            pressed = JoystickControlModes.POSE4_HOLD.value
            self.control_mode = pressed
            if not self.refmodel_req.data:
                self.refmodel_req.data = True
                res = self.refmodel_client(self.refmodel_req.data)
            self.pose4_hold()

        elif buttons["Y"]:
            pressed = JoystickControlModes.DP_CMD.value
            self.control_mode = pressed
            self.pose4_hold()

            self.refmodel_req.data = False
            try:
                res = self.refmodel_client(self.refmodel_req.data)
                if (res.success): rospy.loginfo("Enabling DP command mode...")
                else:
                    rospy.logwarn("Could not contact DP reference model... Enabling manual control.")
                    pressed = JoystickControlModes.OPEN_LOOP.value
                    self.control_mode = pressed
                    self.open_loop()
            except Exception:
                rospy.logwarn("Could not contact DP reference model... Enabling manual control.")
                pressed = JoystickControlModes.OPEN_LOOP.value
                self.control_mode = pressed
                self.open_loop()
                self.refmodel_req.data = True

        if pressed != -1:
            rospy.loginfo(
                f"Control mode changed by joystick: {get_joystick_control_mode_name(pressed)}"
            )
            rospy.sleep(
                rospy.Duration(0.25))  # Sleep to avoid aggressive switching

    @staticmethod
    def killswitch(wrench_publisher_handle):
        rospy.logwarn("KILLSWITCH ENABLED!")

        for i in range(10):
            wrench_publisher_handle.publish(Wrench())
            rospy.sleep(rospy.Duration(0.1))

        nodes = rosnode.get_node_names()
        nodes_to_kill = [
            "/dp_controller", "/thruster_interface", "/pca9685_ros_driver"
        ]
        for node in nodes_to_kill:
            if node in nodes:
                ControlModeHandling.kill_node(node)
            else:
                rospy.loginfo(node + " is not running...")

    @staticmethod
    def emergency_stop(wrench_publisher_handle):
        rospy.logwarn("EMERGENCY STOP ENABLED!")
        nodes = rosnode.get_node_names()
        node = "/dp_controller"
        if node in nodes:
            ControlModeHandling.kill_node(node)
        else:
            rospy.loginfo(node + " is not running...")

        wrench_publisher_handle.publish(Wrench())
        rospy.sleep(rospy.Duration(1.0))

    def open_loop(self):
        dp_goal = dpGoal()
        dp_goal.x_ref = self.odom_pose
        dp_goal.DOF = [0, 0, 0, 0, 0, 0]
        self.dp_client.send_goal(dp_goal)
        rospy.set_param("/DP/Enable", False)

    def pose3_hold(self):
        rospy.set_param("/DP/Enable", True)
        self.send_dp_goal(False)

    def pose4_hold(self):
        rospy.set_param("/DP/Enable", True)
        self.send_dp_goal(True)
    
    def dp_cmd_mode(self, axes):
        wait_time = 0.2
        current_time = rospy.get_time()

        if current_time - self.prev_time < wait_time:
            return

        x = axes["vertical_axis_left_stick"]
        y = axes["horizontal_axis_left_stick"]
        z = -(axes["RT"]-axes["LT"]) / 2
        psi = axes["horizontal_axis_right_stick"]

        # Rotation
        q = self.odom_pose.orientation
        euler = np.array(euler_from_quaternion((q.x, q.y, q.z, q.w)))
        euler[2] = euler[2] + psi
        new_q = quaternion_from_euler(*euler)
        q.x, q.y, q.z, q.w = new_q[0], new_q[1], new_q[2], new_q[3]

        # Position
        R = euler_matrix(*euler)[:3, :3]
        new_pos_body = np.array([x, y, z])
        new_tf = np.dot(R, new_pos_body)

        pos = self.odom_pose.position
        new_x = pos.x + new_tf[0]
        new_y = pos.y + new_tf[1]
        new_z = pos.z + new_tf[2]
        pos.x, pos.y, pos.z = new_x, new_y, new_z

        # Setting up DP target
        dp_cmd_pose = Pose()
        dp_cmd_pose.position = pos
        dp_cmd_pose.orientation = q
        
        self.send_dp_goal(init_z=True, target_pose=dp_cmd_pose)

        self.prev_time = rospy.get_time()

    def send_dp_goal(self, init_z, target_pose=None):
        dp_server_status = self.dp_client.wait_for_server(rospy.Duration(1))

        if not dp_server_status:
            rospy.logwarn("DP Server could not be reached...")
            return

        dp_goal = dpGoal()
        if target_pose is None:
            dp_goal.x_ref = self.odom_pose
        else:
            dp_goal.x_ref = target_pose
        
        if init_z:
            dp_goal.DOF = [1, 1, 1, 0, 0, 1]
        else:
            dp_goal.DOF = [1, 1, 0, 0, 0, 1]

        self.dp_client.send_goal(dp_goal)

    # Function to kill a ROS node
    @staticmethod
    def kill_node(node_name):
        command = ['rosnode', 'kill', node_name]
        subprocess.call(command)
