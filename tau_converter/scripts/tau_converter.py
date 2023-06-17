#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench


class TauRepublisher():
    """Republishes tau with a flipped z axis
    """

    def __init__(self, topic):
        rospy.init_node('tau_republisher_node')

        self.dpTauSub = rospy.Subscriber(topic, Wrench,
                                         self.tau_repub_callback)
        self.tauPub = rospy.Publisher('/thrust/desired_forces/flipped',
                                      Wrench,
                                      queue_size=1)

    def tau_repub_callback(self, msg):
        tau_cmd_msg = Wrench()

        #Setting the message equal to the dp force
        tau_cmd_msg.force = msg.force
        tau_cmd_msg.torque = msg.torque

        #Flipping sign in z
        tau_cmd_msg.force.z = tau_cmd_msg.force.z * (-1)
        self.tauPub.publish(tau_cmd_msg)

if __name__ == '__main__':
    try:
        tau_repub_node = TauRepublisher(topic='/thrust/desired_forces')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
