import rospy
import rosnode
import subprocess

from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Wrench, Pose
from vortex_msgs.msg import dpAction, dpGoal, dpResult

from libjoystick.JoystickControlModes import *

class ControlModeHandling:
    def __init__(self):
        self.control_mode = JoystickControlModes(0)

    def control_mode_change(self, buttons, wrench_publisher_handle, dp_client_handle, odom_pose):
        pressed = -1

        if buttons["stick_button_left"] and buttons["stick_button_right"] and buttons["RB"] and buttons["LB"]:
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
            ControlModeHandling.open_loop(dp_client_handle)

        elif buttons["B"]:
            pressed = JoystickControlModes.POSE3_HOLD.value
            self.control_mode = pressed
            ControlModeHandling.pose3_hold(dp_client_handle, odom_pose)

        elif buttons["X"]:
            pressed = JoystickControlModes.POSE4_HOLD.value
            self.control_mode = pressed
            ControlModeHandling.pose4_hold(dp_client_handle, odom_pose)

        elif buttons["Y"]:
            pressed = JoystickControlModes.DP_CMD.value
            self.control_mode = pressed
            rospy.logwarn("DP_CMD MODE NOT IMPLEMENTED")

        if pressed != -1:
            rospy.loginfo(f"Control mode changed by joystick: {get_joystick_control_mode_name(pressed)}")
            rospy.sleep(rospy.Duration(0.25))  # Sleep to avoid aggressive switching
    
    @staticmethod
    def open_loop(dp_dlient_handler):
        rospy.set_param("/DP/Enable", False)

    @staticmethod
    def killswitch(wrench_publisher_handle):
        rospy.logwarn("KILLSWITCH ENABLED!")
        
        for i in range(10):
            wrench_publisher_handle.publish(Wrench())
            rospy.sleep(rospy.Duration(0.1))

        nodes = rosnode.get_node_names()
        nodes_to_kill = ["/dp_controller", "/thruster_interface", "/pca9685_ros_driver"]
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

    @staticmethod
    def pose3_hold(dp_client, odom_pose):
        dp_pos, dp_q = ControlModeHandling.set_dp_pose(odom_pose)
        ControlModeHandling.send_dp_goal(dp_client, False, odom_pose, dp_pos, dp_q)

    @staticmethod
    def pose4_hold(dp_client, odom_pose):
        dp_pos, dp_q = ControlModeHandling.set_dp_pose(odom_pose)
        ControlModeHandling.send_dp_goal(dp_client, True, odom_pose, dp_pos, dp_q)

    @staticmethod
    def set_dp_pose(odom_pose):

        dp_pos  = [odom_pose.position.x, odom_pose.position.y, odom_pose.position.z]
        dp_q    = (odom_pose.orientation.x,
                   odom_pose.orientation.y,
                   odom_pose.orientation.z,
                   odom_pose.orientation.w)
        
        return dp_pos, dp_q

    @staticmethod
    def send_dp_goal(dp_client, init_z, dp_pose, dp_pos, dp_q):
        rospy.set_param("/DP/Enable", True)
        if init_z:
            rospy.set_param("/setpoint/DOF", [1,1,1,0,0,1])
        else:
            rospy.set_param("/setpoint/DOF", [1,1,0,0,0,1])
        
        dp_rot = euler_from_quaternion(dp_q)
        dp_server_status = dp_client.wait_for_server(rospy.Duration(1))
        
        if not dp_server_status:
            rospy.logwarn("DP Server could not be reached...")
            return

        dp_goal = dpGoal()
        dp_goal.x_ref = dp_pose
        if init_z:
            dp_goal.DOF = [1, 1, 1, 0, 0, 1]
        else:
            dp_goal.DOF = [1, 1, 0, 0, 0, 1]
        
        dp_client.send_goal(dp_goal)

        rospy.set_param("/setpoint/position", [
            float(dp_pos[0]),
            float(dp_pos[1]),
            float(dp_pos[2])
        ])
        rospy.set_param("/setpoint/orientation", dp_rot)
        rospy.loginfo("Holding pose...")

    # Function to kill a ROS node
    @staticmethod
    def kill_node(node_name):
        command = ['rosnode', 'kill', node_name]
        subprocess.call(command)