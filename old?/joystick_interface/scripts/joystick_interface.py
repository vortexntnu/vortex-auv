#!/usr/bin/env python
import rospy
from vortex_msgs.msg import PropulsionCommand, Manipulator
from sensor_msgs.msg import Joy


class JoystickInterfaceNode(object):
    def __init__(self):
        rospy.init_node('joystick_node')

        self.sub = rospy.Subscriber(
            'joy_throttle', Joy, self.callback, queue_size=1)
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,
                                          queue_size=1)
        self.pub_manipulator = rospy.Publisher('manipulator_command',
                                               Manipulator,
                                               queue_size=1)

        # Name buttons and axes based on index from joy-node
        self.buttons_map = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'back',
                            'start', 'power', 'stick_button_left',
                            'stick_button_right']

        self.axes_map = ['horizontal_axis_left_stick',
                         'vertical_axis_left_stick', 'LT',
                         'horizontal_axis_right_stick',
                         'vertical_axis_right_stick', 'RT',
                         'dpad_horizontal', 'dpad_vertical']

    def callback(self, msg):
        # Connect values to names in two dictionaries
        buttons = {}
        axes = {}

        for i in range(len(msg.buttons)):
            buttons[self.buttons_map[i]] = msg.buttons[i]

        for j in range(len(msg.axes)):
            axes[self.axes_map[j]] = msg.axes[j]

        manipulator_msg = Manipulator()
        manipulator_msg.claw_direction = axes['dpad_horizontal']
        manipulator_msg.vertical_stepper_direction = axes['dpad_vertical']

        motion_msg = PropulsionCommand()
        motion_msg.motion = [
            axes['vertical_axis_left_stick'],     # Surge
            -axes['horizontal_axis_left_stick'],  # Sway
            (axes['RT'] - axes['LT'])/2,          # Heave
            (buttons['RB'] - buttons['LB']),      # Roll
            -axes['vertical_axis_right_stick'],   # Pitch
            -axes['horizontal_axis_right_stick']  # Yaw
        ]

        motion_msg.control_mode = [
            (buttons['A'] == 1),
            (buttons['X'] == 1),
            (buttons['B'] == 1),
            (buttons['Y'] == 1),
            (False),
            (False)
        ]

        motion_msg.header.stamp = rospy.get_rostime()

        self.pub_manipulator.publish(manipulator_msg)
        self.pub_motion.publish(motion_msg)


if __name__ == '__main__':
    try:
        joystick_node = JoystickInterfaceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
