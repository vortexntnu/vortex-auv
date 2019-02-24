#!/usr/bin/env python
import rospy
from vortex_msgs.msg import PropulsionCommand
from nav_msgs.msg import Odometry

class LosGuidanceNode(object):
    def __init__(self):
        rospy.init_node('los_guidance_node')

        self.sub = rospy.Subscriber(
            '/odometry/filtered', Odometry, self.callback, queue_size=1)
        self.pub_motion = rospy.Publisher('propulsion_command',
                                          PropulsionCommand,
                                          queue_size=1)


    def callback(self, msg):
        print('Message recieved LOS LOS')
        motion_msg = PropulsionCommand()
        motion_msg.motion = [
            0,     # Surge
            0,  # Sway
            0,          # Heave
            0,      # Roll
            0,   # Pitch
            0.5  # Yaw
        ]
	
        motion_msg.control_mode = [
            (False),
            (False),
            (False),
            (False),
            (False),
            (False)
        ]

        motion_msg.header.stamp = rospy.get_rostime()
        self.pub_motion.publish(motion_msg)



if __name__ == '__main__':
    try:
        los_guidance_node = LosGuidanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
