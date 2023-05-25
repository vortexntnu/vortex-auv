#! /usr/bin/env python3

import math
import numpy as np


def distance_between_points(p1, p2):
    return math.dist([p1.x, p1.y, p1.z], [p2.x, p2.y, p2.z])


# Checks if we are above docking station within the radius of acceptance
def above_docking_point(max_error, own_pose, goal_pose):
    error = np.sqrt(
        pow(own_pose.position.x - goal_pose.position.x, 2) +
        pow(own_pose.position.y - goal_pose.position.y, 2))
    if (error < max_error):
        return True
    return False


# Check if new estimate is far enough away from old estimate to justify updating DP setpoint
def should_send_new_goal(self, error_coefficients, object_pose,
                         current_goal_pose, own_pose):
    error = distance_between_points(object_pose.position,
                                    current_goal_pose.position)

    a, b = error_coefficients['a'], error_coefficients['b']
    error_limit = a * distance_between_points(own_pose.position,
                                              object_pose.position) + b
    if error > error_limit:
        return True
    return False
