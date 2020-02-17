#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from vortex_msgs.srv import LandmarkPose, LandmarkPoseResponse


# TODO: actual landmark positions should be found by perception
gate_found = True
gate_pose = Pose(
    Point(10, 2, -2), 
    Quaternion(*quaternion_from_euler(0, 0, 0))
)
buoy_found = True
buoy_pose = Pose(
    Point(-10, 0, -2), 
    Quaternion(*quaternion_from_euler(0, 0, 0))
)


def landmark_cb(req):
    """
    checks if gives landmark is found and returns it's Pose.
    Pose (and not point) because the objects orientation can be important. 
    """

    if req.landmark == 'GATE':
        return LandmarkPoseResponse(
            gate_found,
            gate_pose
        )

    elif req.landmark == 'BUOY':
        return LandmarkPoseResponse(
            buoy_found,
            buoy_pose
        )

    else:
        rospy.logerr('Unknown landmark requested of landmarks service')


def landmark_server():
    rospy.init_node('landmark_server')
    landmark_service = rospy.Service('landmarks', LandmarkPose, landmark_cb)
    rospy.loginfo('landmark service running')
    rospy.spin()


if __name__ == "__main__":
    landmark_server()
