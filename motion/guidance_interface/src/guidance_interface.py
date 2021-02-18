#!/usr/bin/env python
# coding: UTF-8

"""

Node som forenkler tilganger til controlleren. 
Den skal ta av seg bytting mellom controller moduser, 
slik at koden i state machinene kan bli enklere. 

Noden skal ha en action-server som tar inn en ny action (ligner på
move_base) som skal inneholde: 
- Target pose og 
- String for kontroller som skal benyttes

Noden sender så target posen videre som en ny action
til den valgte kontrolleren. Resultat og feedback fra endelig kontroller
sendes videre igjennom noden. 

"""


import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Joy
from vortex_msgs.msg import (
    MoveAction, MoveActionFeedback, MoveActionResult, LosPathFollowingAction, LosPathFollowingGoal
)
from vortex_msgs.srv import ControlMode


# ENUM for controller mode
OPEN_LOOP           = 0
POSE_HOLD           = 1
HEADING_HOLD        = 2
DEPTH_HEADING_HOLD  = 3
DEPTH_HOLD          = 4
POSE_HEADING_HOLD   = 5
CONTROL_MODE_END    = 6


def change_control_mode(requested_mode):
    """
    Change the controller mode of the DP controller.

    Although this is technically breaking the design
    idea that FSM and controller should not talk to
    eachother, it makes sense to keep it for now, seeing
    as some guidance systems may rely on one of the control
    modes for the controller.

    In the future, it may be moved to the DP guidance node,
    if it makes sense to.

    """

    rospy.wait_for_service('/controller/controlmode_service')   # From controller_ros.cpp

    try:
        control_mode = rospy.ServiceProxy('/controller/controlmode_service', ControlMode)
        response = control_mode(requested_mode)
        return response.result

    except rospy.ServiceException as e:
        rospy.logerr('guidance_interface could not change control mode')
        print('Service call failed: %s' %e)


class GuidanceInterface:

    def __init__(self):
        """
        Define constants used in the guidance systems, create the
        move action server that the fsm uses to communicate with 
        this node,  and connect to the actions servers in the guidance
        systems.
        """

        self.joystick_surge_scaling = rospy.get_param('/joystick/scaling/surge')
        self.joystick_sway_scaling  = rospy.get_param('/joystick/scaling/sway')
        self.joystick_heave_scaling = rospy.get_param('/joystick/scaling/heave')
        self.joystick_roll_scaling  = rospy.get_param('/joystick/scaling/roll')
        self.joystick_pitch_scaling = rospy.get_param('/joystick/scaling/pitch')
        self.joystick_yaw_scaling   = rospy.get_param('/joystick/scaling/yaw')

        # Name buttons and axes based on index from joy-node
        self.joystick_buttons_map = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'back',
							         'start', 'power', 'stick_button_left',
							         'stick_button_right']

        self.joystick_axes_map = ['horizontal_axis_left_stick',
						          'vertical_axis_left_stick', 'LT',
						          'horizontal_axis_right_stick',
						          'vertical_axis_right_stick', 'RT',
						          'dpad_horizontal', 'dpad_vertical']

        
        self.transit_speed = rospy.get_param('~transit_speed', 0.3)
        self.sphere_of_acceptance = rospy.get_param('~sphere_of_acceptance', 0.5)
        self.timeout = rospy.get_param('~guidance_interface_timeout', 90)

        self.action_server = actionlib.SimpleActionServer('move', MoveAction, self.move_cb, auto_start=False)
        self.action_server.start()

        self.dp_client = actionlib.SimpleActionClient('dp_action_server', MoveBaseAction)
        self.los_client = actionlib.SimpleActionClient('los_action_server', LosPathFollowingAction)

        
        self.joystick_sub = rospy.Subscriber('/joystick/joy', Joy, self.joystick_cb, queue_size=1)
        self.joystick_pub = rospy.Publisher('/guidance/joystick_data', Joy, queue_size=1)

        rospy.loginfo('Guidance interface is up and running')


    def move_cb(self, move_goal):
        """
        Converts move_goal into the proper goal type for the desired guidance
        system and engages the corresponding guidance node through the
        action servers.

        Aborts action if it is not completed within self.timeout seconds
        """

        # Talk to the dp_guidance node...
        if move_goal.guidance_type == 'PositionHold':
            rospy.loginfo('move_cb -> PositionHold. Changing control mode...')
            change_control_mode(POSE_HEADING_HOLD)

            dp_goal = MoveBaseGoal()
            dp_goal.target_pose.pose = move_goal.target_pose

            self.dp_client.send_goal(dp_goal, done_cb=self.done_cb, feedback_cb=None)

            if not self.dp_client.wait_for_result(timeout=rospy.Duration(self.timeout)):
                self.action_server.set_aborted()
                rospy.loginfo('DP guidance aborted action due to timeout')


        # Talk to the los_guidance node...
        elif move_goal.guidance_type == 'LOS':
            rospy.loginfo('move_cb -> LOS. Changing control mode...')
            change_control_mode(OPEN_LOOP)

            los_goal = LosPathFollowingGoal()
            los_goal.next_waypoint = move_goal.target_pose.position
            los_goal.forward_speed.linear.x = self.transit_speed
            los_goal.sphereOfAcceptance = self.sphere_of_acceptance
            los_goal.desired_depth.z = move_goal.target_pose.position.z

            self.los_client.send_goal(los_goal, done_cb=self.done_cb, feedback_cb=None)

            if not self.los_client.wait_for_result(timeout=rospy.Duration(self.timeout)):
                self.action_server.set_aborted()
                rospy.loginfo('LOS guidance aborted action due to timeout')
        
        else:
            rospy.logerr('Unknown guidace type sent to guidance_interface')
            self.action_server.set_aborted()

    
    def done_cb(self, state, result):
        """
        Set the outcome of the action depending on
        the returning result.
        """
        
        if state == GoalStatus.SUCCEEDED:
            self.action_server.set_succeeded()

        elif state == GoalStatus.PREEMPTED:
            self.action_server.set_preempted()

        else:
            self.action_server.set_aborted()

    def joystick_cb(self, msg):
        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map[i]] = msg.buttons[i]

        for j in range(len(msg.axes)):
            axes[self.joystick_axes_map[j]] = msg.axes[j]

        # TODO: Buttons to change control mode, should be a service server between this and guidance

        surge 	= axes['vertical_axis_left_stick'] * self.joystick_surge_scaling
        sway 	= axes['horizontal_axis_left_stick'] * self.joystick_sway_scaling
        heave 	= (axes['RT'] - axes['LT'])/2 * self.joystick_heave_scaling
        roll 	= (buttons['RB'] - buttons['LB']) * self.joystick_roll_scaling  
        pitch 	= axes['vertical_axis_right_stick'] * self.joystick_pitch_scaling
        yaw 	= axes['horizontal_axis_right_stick'] * self.joystick_yaw_scaling

        joystick_msg = Joy()
        joystick_msg.axes = [surge, sway, heave, roll, pitch, yaw]

        self.joystick_pub.publish(joystick_msg)

if __name__ == "__main__":
    rospy.init_node('interface')
    server = GuidanceInterface()
    rospy.spin()
