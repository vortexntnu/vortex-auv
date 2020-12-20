#!/usr/bin/env python
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.

import rospy

class DPGuidance:
    """
    The dynamic positioning guidance system is very simple.
    It does not perform any calculations, but simply passes
    the setpoints directly to the DP controller in the
    /controller/dp node.
    """

    def __init__(self):

        rospy.init_node('dp')
        



if __name__ == '__main__':
    
    try:
        dp_guidance = DPGuidance()
        rospy.spin()

    except rospy.ROSInternalException as e:
        rospy.logerr(e)