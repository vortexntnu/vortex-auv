#!/usr/bin/env python

import rospy
import actionlib

from sensor_msgs.msg import Joy
from vortex_msgs.msg import ControlModeAction

class JoystickInterface():


    def __init__(self):
        """
        Define constants used in the joystick mapping, and any ros
        specifics
        """

        rospy.init_node('joystick_interface')

        self.joystick_surge_scaling = rospy.get_param('/joystick/scaling/surge')
        self.joystick_sway_scaling  = rospy.get_param('/joystick/scaling/sway')
        self.joystick_heave_scaling = rospy.get_param('/joystick/scaling/heave')
        self.joystick_roll_scaling  = rospy.get_param('/joystick/scaling/roll')
        self.joystick_pitch_scaling = rospy.get_param('/joystick/scaling/pitch')
        self.joystick_yaw_scaling   = rospy.get_param('/joystick/scaling/yaw')

        self.joystick_buttons_map = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'back',
							         'start', 'power', 'stick_button_left',
							         'stick_button_right']

        self.joystick_axes_map = ['horizontal_axis_left_stick',
						          'vertical_axis_left_stick', 'LT',
						          'horizontal_axis_right_stick',
						          'vertical_axis_right_stick', 'RT',
						          'dpad_horizontal', 'dpad_vertical']

        

        self.joystick_sub = rospy.Subscriber('/joystick/joy', Joy, self.joystick_cb, queue_size=1)
        self.joystick_pub = rospy.Publisher('/mission/joystick_data', Joy, queue_size=1)

        self.guidance_interface_client = actionlib.SimpleActionClient("/guidance/control_mode_server", ControlModeAction)

        rospy.loginfo('Joystick interface is up and running')

    def joystick_cb(self, msg):
        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.joystick_buttons_map[i]] = msg.buttons[i]

        for i in range(len(msg.axes)):
            axes[self.joystick_axes_map[i]] = msg.axes[i]


        abxy = self._abxy_pressed(buttons)
        if abxy != -1:
            self.guidance_interface_client.send_goal(abxy)

        surge 	= axes['vertical_axis_left_stick'] * self.joystick_surge_scaling
        sway 	= axes['horizontal_axis_left_stick'] * self.joystick_sway_scaling
        heave 	= (axes['RT'] - axes['LT'])/2 * self.joystick_heave_scaling
        roll 	= (buttons['RB'] - buttons['LB']) * self.joystick_roll_scaling  
        pitch 	= axes['vertical_axis_right_stick'] * self.joystick_pitch_scaling
        yaw 	= axes['horizontal_axis_right_stick'] * self.joystick_yaw_scaling

        joystick_msg = Joy()
        joystick_msg.axes = [surge, sway, heave, roll, pitch, yaw]
        
        self.joystick_pub.publish(joystick_msg)

    def _abxy_pressed(self, buttons):
        for i in range(4):
            if buttons[i]:
                return i

        return -1 # Needs to be -1 since 0 (or "false") is part of the valid indices above

if __name__ == '__main__':
    
    try:
        joystick_interface = JoystickInterface()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass