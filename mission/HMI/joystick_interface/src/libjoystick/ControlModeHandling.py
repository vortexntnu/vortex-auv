import rospy
import rosnode
import actionlib
import subprocess

from geometry_msgs.msg import Wrench, Pose
from vortex_msgs.msg import dpAction, dpGoal, dpResult

from libjoystick.JoystickControlModes import *


class ControlModeHandling:

    def __init__(self):
        self.control_mode = JoystickControlModes(0)
        self.odom_pose = Pose()
        
        # DP server and client
        dp_action_server = "/DpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                      dpAction)

        # self.dp_result = rospy.Subscriber("/DpAction/result", dpResult, self.dp_goal_cb)

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
            self.open_loop()

        elif buttons["B"]:
            pressed = JoystickControlModes.POSE3_HOLD.value
            self.control_mode = pressed
            self.pose3_hold()

        elif buttons["X"]:
            pressed = JoystickControlModes.POSE4_HOLD.value
            self.control_mode = pressed
            self.pose4_hold()

        elif buttons["Y"]:
            pressed = JoystickControlModes.DP_CMD.value
            self.control_mode = pressed
            rospy.set_param("/DP/Enable", False)
            rospy.logwarn("DP_CMD MODE NOT IMPLEMENTED")

        if pressed != -1:
            rospy.loginfo(
                f"Control mode changed by joystick: {get_joystick_control_mode_name(pressed)}"
            )
            rospy.sleep(
                rospy.Duration(0.25))  # Sleep to avoid aggressive switching

    def open_loop(self):
        dp_goal = dpGoal()
        dp_goal.x_ref = self.odom_pose
        dp_goal.DOF = [0, 0, 0, 0, 0, 0]
        self.dp_client.send_goal(dp_goal)
        rospy.set_param("/DP/Enable", False)

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

    def pose3_hold(self):
        self.send_dp_goal(False)

    def pose4_hold(self):
        self.send_dp_goal(True)

    def send_dp_goal(self, init_z):
        rospy.set_param("/DP/Enable", True)

        dp_server_status = self.dp_client.wait_for_server(rospy.Duration(1))

        if not dp_server_status:
            rospy.logwarn("DP Server could not be reached...")
            return

        dp_goal = dpGoal()
        dp_goal.x_ref = self.odom_pose
        if init_z:
            dp_goal.DOF = [1, 1, 1, 0, 0, 1]
        else:
            dp_goal.DOF = [1, 1, 0, 0, 0, 1]

        self.dp_client.send_goal(dp_goal)

        rospy.loginfo("Holding pose...")

    # Function to kill a ROS node
    @staticmethod
    def kill_node(node_name):
        command = ['rosnode', 'kill', node_name]
        subprocess.call(command)
