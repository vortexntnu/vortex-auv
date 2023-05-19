#! /usr/bin/env python3

import rospy
import tf

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

from libwaypointinterface.DPClient import DPClient


class WaypointInterface:
    """
    The WaypointInterface class processes incoming waypoints from operator interfaces,
    such as RViz and FoxGlove-Studio, and sends navigational goals to a guidance module (DP currently). 
    """

    def __init__(self):
        rospy.init_node('waypoint_interface', anonymous=False)

        # Attributes
        self.init_z = False  # If Z-DOF is initiated, keep it so

        self.nav_goal = PoseStamped()
        self.nav_goal_z = 0.0

        # TF
        self.tf_listener = tf.TransformListener()

        # Subscribers
        rospy.Subscriber("/move_base_simple/goal",
                         PoseStamped,
                         self.nav_goal_cb,
                         queue_size=1)
        rospy.Subscriber("/move_base_simple/goal/z",
                         Float32,
                         self.nav_goal_z_cb,
                         queue_size=1)

        self.dp_client = DPClient()

        rospy.loginfo("Waypoint Interface Initialized!")

    def nav_goal_to_dp_goal(self, enable_z=False):
        """
        Converts the current navigation goal to a DP goal and sends it to the DPClient.

        Args:
            enable_z (bool, default=False): Indicates whether to adjust the altitude of the DP goal.
        
        Returns:
            bool: Stops and returns 'False' is 'odom' TF is not available.
        """
        if not self.dp_client.get_enabled_status:
            self.dp_client.enable()

        self.dp_client.goal.DOF = [1, 1, 0, 0, 0, 1]

        try:
            self.tf_listener.waitForTransform('odom',
                                              self.nav_goal.header.frame_id,
                                              rospy.Time(),
                                              rospy.Duration(4.0))
            self.dp_client.goal.x_ref = self.tf_listener.transformPose(
                'odom', self.nav_goal).pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn("Could not retreive transform to 'odom' frame...")
            return False

        if enable_z or self.init_z:
            self.dp_client.goal.DOF = [1, 1, 1, 0, 0, 1]
            self.init_z = True

            self.dp_client.goal.x_ref.position.z = self.nav_goal_z

        return True

    def nav_goal_cb(self, posestamped_msg):
        """
        Callback function for "/move_base_simple/goal" topic.
        Convert provided navigation goal message to a DP goal.
        Sends the goal to DP server.

        Args:
            posestamped_msg (PoseStamped): The received PoseStamped message.
        """
        self.nav_goal = posestamped_msg
        self.nav_goal_to_dp_goal()
        self.dp_client.send_goal()

    def nav_goal_z_cb(self, float32_msg):
        """
        Callback function for "/move_base_simple/goal/z" topic. 
        Adds provided Z setpoint to the current Pose goal.
        Sends the modified goal to DP server.

        Args:
            float32_msg (Float32): The received Float32 message.
        """
        self.nav_goal_z = float32_msg.data
        self.nav_goal_to_dp_goal(enable_z=True)
        self.dp_client.send_goal()
