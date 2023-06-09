#! /usr/bin/env python3

import rospy

from libdockingfsm.DockingFSM import DockingFSM

if __name__ == "__main__":
    docking_fsm = DockingFSM()
    if not rospy.is_shutdown():
        docking_fsm.main()
