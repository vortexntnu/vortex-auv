#!/usr/bin/env python

import rospy
from vortex_msgs.srv import ControlMode

#ENUM
OPEN_LOOP           = 0
POSE_HOLD           = 1
HEADING_HOLD        = 2
DEPTH_HEADING_HOLD  = 3
DEPTH_HOLD          = 4
STAY_LEVEL          = 5
CONTROL_MODE_END    = 6


class ControllerMode:


    def __init__(self):
    	#pass
        #print('init')
        rospy.init_node('change_control_mode')


    def change_control_mode_client(self,requested_mode):

        rospy.wait_for_service('controlmode_service')
        try:
            control_mode = rospy.ServiceProxy('controlmode_service', ControlMode)
            response = control_mode(requested_mode)
            print(response.result)
            return response.result
        except rospy.ServiceException, e:
            print "Service call failed"
    

if __name__ == '__main__':
    
    try:
        control_mode = ControllerMode()
        control_mode.change_control_mode_client(POSE_HOLD)
        rospy.spin()

    except rospy.ROSInterruptException:
        print('caught exeption')
        pass

