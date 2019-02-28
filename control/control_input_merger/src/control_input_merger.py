#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Wrench
from vortex_msgs.msg import PropulsionCommand

class ControlInputMerger(object):
    def __init__(self):
        rospy.init_node('control_input_merger_node')

        #Subscriber nodes
        self.sub_surge = rospy.Subscriber('surge_input', Wrench, self.surge_callback, queue_size=1)
        self.sub_sway = rospy.Subscriber('sway_input', Wrench, self.sway_callback, queue_size=1)
        self.sub_heave = rospy.Subscriber('heave_input', Wrench, self.heave_callback, queue_size=1)
        self.sub_roll = rospy.Subscriber('roll_input',Wrench, self.yaw_callback, queue_size=1)
        self.sub_pitch = rospy.Subscriber('pitch_input',Wrench, self.yaw_callback, queue_size=1)
        self.sub_yaw = rospy.Subscriber('yaw_input',Wrench, self.yaw_callback, queue_size=1)

        #Publisher node
        self.pub_thrust = rospy.Publisher('/manta/thruster_manager/input', Wrench, queue_size=1)


        #Initialize thrust publisher message
        self.motion_msg = Wrench()

    #Callback
    def surge_callback(self, msg):
        self.motion_msg.force.x = msg.force.x
        self.pub_thrust.publish(self.motion_msg)


    def sway_callback(self, msg):
        self.motion_msg.force.y = msg.force.y
        self.pub_thrust.publish(self.motion_msg)

    def heave_callback(self, msg):
        self.motion_msg.force.z = msg.force.z
        self.pub_thrust.publish(self.motion_msg)

    def roll_callback(self, msg):
        self.motion_msg.torque.x = msg.torque.x
        self.pub_thrust.publish(self.motion_msg)

    def pitch_callback(self, msg):
        self.motion_msg.torque.y = msg.torque.y
        self.pub_thrust.publish(self.motion_msg)

    def yaw_callback(self, msg):
        self.motion_msg.torque.z = msg.torque.z
        self.pub_thrust.publish(self.motion_msg)




if __name__ == '__main__':
    try:
        control_input_merger_node = ControlInputMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass