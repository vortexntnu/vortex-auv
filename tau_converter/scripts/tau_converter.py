#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench


class TauRepublisher():
    """Republishes tau from DP controller
    """

    def __init__(self, topic):
        rospy.init_node('tau_republisher_node')

        self.tau_cmd_msg = Wrench()
        self.dpTauSub = rospy.Subscriber(topic, Wrench,
                                         self.tau_repub_callback)
        self.tauPub = rospy.Publisher('/thrust/desired_forces/flipped',
                                      Wrench,
                                      queue_size=1)

    def tau_repub_callback(self, msg):

        #Setting the message equal to the dp force
        self.tau_cmd_msg.force = msg.force
        self.tau_cmd_msg.torque = msg.torque

        #Flipping sign in z
        self.tau_cmd_msg.force.z = self.tau_cmd_msg.force.z * (-1)

    def spin(self):

        while not rospy.is_shutdown():
            self.tauPub.publish(self.tau_cmd_msg)


if __name__ == '__main__':
    try:
        tau_repub_node = TauRepublisher(topic='/thrust/desired_forces')
        tau_repub_node.spin()

    except rospy.ROSInterruptException:
        pass
