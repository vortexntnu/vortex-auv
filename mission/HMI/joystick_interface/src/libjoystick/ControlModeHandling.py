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
    """
    Class for handling control mode changes using joystick inputs.

    Attributes:
        control_mode    (IntEnum): Current control mode setting.
        odom_pose       (Pose): Drone pose in the odom frame. 
        prev_time       (float): Previously noted time. Used in dp_cmd_mode only.

        dp_client       (SimpleActionClient): DP controller action client.
        refmodel_client (ServiceProxy): DP reference model service client.
        refmodel_req    (SetBoolRequest): SetBool service request object. Used for the DP reference model client.
    """

    def __init__(self):
        """
        Initializes the control mode handler - attributes and ROS.
        """
        self.control_mode = JoystickControlModes(0)  # Initial control mode
        self.odom_pose = Pose()  # Pose from odometry
        self.prev_dp_cmd_pose = Pose()  # Pose from prev DP command
        self.prev_time = 0.0  # Used for time interval calculations

        # Initialize dynamic positioning (DP) client
        self.dp_client = actionlib.SimpleActionClient("/DpAction", dpAction)

        # Initialize reference model service client
        self.refmodel_client = rospy.ServiceProxy(
            '/dp_reference_model/enable_x_ref_filter', SetBool)

        # Initialize reference model service request
        self.refmodel_req = SetBoolRequest()
        self.refmodel_req.data = True  # Enable reference model by default

    def control_mode_change(self, buttons, wrench_publisher_handle):
        """
        Changes the control mode based on joystick button inputs.

        Args:
            buttons (dict): A dictionary mapping buttons to their state.
            wrench_publisher_handle: Publisher to publish Wrench messages.
        """
        # Checks if a control mode changing button has been pressed
        is_pressed = False

        # Handles killswitch mode
        if buttons["stick_button_left"] and buttons[
                "stick_button_right"] and buttons["RB"] and buttons["LB"]:
            is_pressed = True
            self.control_mode = JoystickControlModes.KILLSWITCH.value
            ControlModeHandling.killswitch(wrench_publisher_handle)

        # Handles emergency stop
        if buttons["start"]:
            is_pressed = True
            self.control_mode = JoystickControlModes.EMERGENCY_STOP.value
            ControlModeHandling.emergency_stop(wrench_publisher_handle)

        # Handles open-loop mode
        elif buttons["A"]:
            is_pressed = True
            self.control_mode = JoystickControlModes.OPEN_LOOP.value
            if not self.refmodel_req.data:
                self.refmodel_req.data = True
                res = self.refmodel_client(self.refmodel_req.data)
            self.open_loop()

        # Handles DP hold in x, y, and yaw
        elif buttons["B"]:
            is_pressed = True
            self.control_mode = JoystickControlModes.POSE3_HOLD.value
            if not self.refmodel_req.data:
                self.refmodel_req.data = True
                res = self.refmodel_client(self.refmodel_req.data)
            self.pose3_hold()

        # Handles DP hold in x, y, z, and yaw
        elif buttons["X"]:
            is_pressed = JoystickControlModes.POSE4_HOLD.value
            self.control_mode = is_pressed
            if not self.refmodel_req.data:
                self.refmodel_req.data = True
                res = self.refmodel_client(self.refmodel_req.data)
            self.pose4_hold()

        # Handles DP control mode - control drone using DP from joystick
        elif buttons["Y"]:
            is_pressed = True
            self.control_mode = JoystickControlModes.DP_CMD.value
            self.pose4_hold()

            self.refmodel_req.data = False
            try:
                res = self.refmodel_client(self.refmodel_req.data)
                if (res.success): rospy.loginfo("Enabling DP command mode...")
                else:
                    rospy.logwarn(
                        "Could not contact DP reference model... Enabling manual control."
                    )
                    is_pressed = JoystickControlModes.OPEN_LOOP.value
                    self.control_mode = is_pressed
                    self.open_loop()
            except Exception:
                rospy.logwarn(
                    "Could not contact DP reference model... Enabling manual control."
                )
                is_pressed = JoystickControlModes.OPEN_LOOP.value
                self.control_mode = is_pressed
                self.open_loop()
                self.refmodel_req.data = True

        if is_pressed == True:
            rospy.loginfo(
                f"Control mode changed by joystick: {get_joystick_control_mode_name(self.control_mode)}"
            )
            rospy.sleep(
                rospy.Duration(0.25))  # Sleep to avoid aggressive switching

    @staticmethod
    def killswitch(wrench_publisher_handle):
        """
        Activates the killswitch. Currently limited to just killing ROS nodes that are required to move the drone.
        Publishes empty Wrench messages to stop all motion.

        Args:
            wrench_publisher_handle: Publisher to publish Wrench messages.
        """
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
        """
        Activates the emergency stop. Kills the dynamic positioning controller.

        Args:
            wrench_publisher_handle: Publisher to publish Wrench messages.
        """
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
        """
        Enables open loop control. Sends goal to dynamic positioning action server.
        """
        dp_goal = dpGoal()
        dp_goal.x_ref = self.odom_pose
        dp_goal.DOF = [0, 0, 0, 0, 0, 0]
        self.dp_client.send_goal(dp_goal)
        rospy.set_param("/DP/Enable", False)

    def pose3_hold(self):
        """
        Enables pose hold in 3 degrees of freedom: x, y, psi
        """
        rospy.set_param("/DP/Enable", True)
        self.send_dp_goal(False)

    def pose4_hold(self):
        """
        Enables pose hold in 4 degrees of freedom: x, y, z, psi
        """
        rospy.set_param("/DP/Enable", True)
        self.send_dp_goal(True)

    def dp_cmd_mode(self, axes):
        """
        Enables dynamic positioning command mode.

        Args:
            axes (dict): A dictionary mapping joystick axes to their state.
        """
        wait_time = 0.2
        current_time = rospy.get_time()
        threshold = 0.05

        if current_time - self.prev_time < wait_time:
            return

        x = axes["vertical_axis_left_stick"]
        y = axes["horizontal_axis_left_stick"]
        z = -(axes["RT"] - axes["LT"]) / 2
        psi = axes["horizontal_axis_right_stick"]

        # If no new goal, hold previous pose goal
        if (x < threshold and x > -threshold) and (y < threshold and y > -threshold) and (
                z < threshold and z > -threshold) and (psi < threshold and psi > -threshold):
            self.send_dp_goal(init_z=True, target_pose=self.prev_dp_cmd_pose)
            self.prev_time = rospy.get_time()
            return

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

        # If no new psi goal, hold previous psi goal
        if (psi < threshold and psi > -threshold):
            dp_cmd_pose.orientation = self.prev_dp_cmd_pose.orientation

        # If no new x,y goal, hold previous x,y goal
        if (x < threshold and x > -threshold) and (y < threshold and y > -threshold):
            dp_cmd_pose.position.x = self.prev_dp_cmd_pose.position.x
            dp_cmd_pose.position.y = self.prev_dp_cmd_pose.position.y

        # If no new z goal, hold previous z goal
        if (z < threshold and z > -threshold):
            dp_cmd_pose.position.z = self.prev_dp_cmd_pose.position.z

        self.send_dp_goal(init_z=True, target_pose=dp_cmd_pose)

        # Save current DP goal and time for next iter
        self.prev_dp_cmd_pose = dp_cmd_pose
        self.prev_time = rospy.get_time()

    def send_dp_goal(self, init_z, target_pose=None):
        """
        Sends a goal to the dynamic positioning action server.

        Args:
            init_z (bool): Initialize z-axis control.
            target_pose (Pose, optional): Target pose for the dynamic positioning controller.
        """
        dp_server_status = self.dp_client.wait_for_server(rospy.Duration(1))

        if not dp_server_status:
            rospy.logwarn("DP Server could not be reached...")
            return

        dp_goal = dpGoal()
        if target_pose is None:
            self.prev_dp_cmd_pose = self.odom_pose
            dp_goal.x_ref = self.odom_pose
        else:
            dp_goal.x_ref = target_pose

        if init_z:
            dp_goal.DOF = [1, 1, 1, 0, 0, 1]
        else:
            dp_goal.DOF = [1, 1, 0, 0, 0, 1]

        self.dp_client.send_goal(dp_goal)

    @staticmethod
    def kill_node(node_name):
        """
        Kills a ROS node.

        Args:
            node_name (str): Name of the ROS node to kill.
        """
        command = ['rosnode', 'kill', node_name]
        subprocess.call(command)
