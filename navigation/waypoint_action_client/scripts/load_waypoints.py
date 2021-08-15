#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import os
import rospy
import numpy as np
import yaml
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

        #   "atan2"
        #            y 90
        #     180    |     0         ^
        #        <---*----> x   rot+ |
        #    -180    |     0
        #           -90
        #

class PrepareWaypoints:

    def __init__(self, inertial_frame_id='world'):

        # Waypoints INIT
        assert inertial_frame_id in ['world', 'world_enu']
        self._inertial_frame_id = inertial_frame_id
        self.waypoints = []

    def read_from_file(self, filename):
        if not os.path.isfile(filename):
            print 'Invalid waypoint filename, file', filename
            return False
        try:
            with open(filename, 'r') as wp_file:
                wps = yaml.load(wp_file)

                # check if .yaml has a list
                if isinstance(wps['waypoints'], list):
                    
                    # pick every waypoint an insert into array
                    for wp in wps['waypoints']:
                        x = wp['point'][0]
                        y = wp['point'][1]
                        z = wp['point'][2]
                        #yaw = wp['R']
                        #self.waypoints.append((x,y,z,yaw))
                        R = wp['RPY'][0]
                        P = wp['RPY'][1]
                        Y = wp['RPY'][2]
			self.waypoints.append((x,y,z,R,P,Y))			
                    #print(self.waypoints)

        except Exception, e:
            print'Error when loading the waypoint file'
            print str(e)
            return False
        return True
        
    def delete_waypoint(self,waypoints):
        # pop first waypoint in list
        waypoints.pop(0)
        
"""
# ROS spin
if __name__ == '__main__':

    try:

    	# ROS INIT
        rospy.init_node('load_waypoints')
        rate = rospy.Rate(20) #20hz 
        # get file
        if rospy.is_shutdown():
                rospy.logerr('ROS master not running!')
                sys.exit(-1)
                rospy.logerr('Found waypoint file!')
        if rospy.has_param('~filename'):

                filename = rospy.get_param('~filename')
        else:
                raise rospy.ROSException('No filename found')
        
        # Start tracking by reading waypoints from file
        load = PrepareWaypoints()
        load.read_from_file(filename)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exeption')
    print('exiting')

"""
