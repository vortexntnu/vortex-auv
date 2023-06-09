#! /usr/bin/env python3

import rospy

from libwaypointinterface.WaypointInterface import WaypointInterface

if __name__ == '__main__':
    wp_interface = WaypointInterface()

    while not rospy.is_shutdown():
        rospy.spin()
