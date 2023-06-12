#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench

class TauSub():
    """
    Node for printing output forces from flipped tau
    """

    def __init__(self, topic):
        rospy.init_node('tau_sub_node')
        self.tau_cmd_msg = Wrench
        self.dpTauSub = rospy.Subscriber(topic, Wrench, self.tau_sub_callback)

    def tau_sub_callback(self, msg):
        rospy.loginfo(msg)

if __name__ == '__main__':
    try:
        tau_sub_node = TauSub(
            topic='/thrust/desired_forces/flipped')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
