#! /usr/bin/env python3

import rospy
from Docking import Docking

if __name__ == "__main__":
    docking_fsm = Docking()
    if not rospy.is_shutdown():
        docking_fsm.main()
