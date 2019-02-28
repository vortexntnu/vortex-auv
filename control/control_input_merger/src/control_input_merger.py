#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench
from vortex_msgs.msg import PropulsionCommand

class ControlInputMerger(object):
    def __init__(self):
        rospy.init_node('control_input_merger_node')

        #Subscriber nodes
        self.sub_surge = rospy.Subscriber('surge_input', PropulsionCommand, self.surge_callback, queue_size=1)
        self.sub_sway = rospy.Subscriber('sway_input', PropulsionCommand, self.sway_callback, queue_size=1)
        self.sub_heave = rospy.Subscriber('heave_input', PropulsionCommand, self.heave_callback, queue_size=1)
        self.sub_roll = rospy.Subscriber('roll_input',PropulsionCommand, self.yaw_callback, queue_size=1)
        self.sub_pitch = rospy.Subscriber('pitch_input',PropulsionCommand, self.yaw_callback, queue_size=1)
        self.sub_yaw = rospy.Subscriber('yaw_input',PropulsionCommand, self.yaw_callback, queue_size=1)

        #Publisher node
        self.pub_thrust = rospy.Publisher('propulsion_command', PropulsionCommand, queue_size=1)


        #Initialize thrust publisher message
        self.motion_msg = PropulsionCommand()

        self.motion_msg.control_mode = [
            (False),
            (False),
            (False),
            (False),
            (False),
            (False)
        ]
        self.motion_msg.motion = [
            0,  # Surge
            0,  # Sway
            0,  # Heave
            0,  # Roll
            0,  # Pitch
            0   # Yaw
        ]

    #Callback
    def surge_callback(self, msg):
        self.motion_msg.motion[0] = msg.motion[0]
        self.pub_thrust.publish(self.motion_msg)


    def sway_callback(self, msg):
        self.motion_msg.motion[1] = msg.motion[1]
        self.pub_thrust.publish(self.motion_msg)

    def heave_callback(self, msg):
        self.motion_msg.motion[2] = msg.motion[2]
        self.pub_thrust.publish(self.motion_msg)

    def roll_callback(self, msg):
        self.motion_msg.motion[3] = msg.motion[3]
        self.pub_thrust.publish(self.motion_msg)

    def pitch_callback(self, msg):
        self.motion_msg.motion[4] = msg.motion[4]
        self.pub_thrust.publish(self.motion_msg)

    def yaw_callback(self, msg):
        self.motion_msg.motion[5] = msg.motion[5]
        self.pub_thrust.publish(self.motion_msg)




if __name__ == '__main__':
    try:
        control_input_merger_node = ControlInputMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass