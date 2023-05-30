#! /usr/bin/env python3

import math
import numpy as np


def distance_between_points(p1, p2, convergence_height=0.0):
    return math.dist([p1.x, p1.y, p1.z + convergence_height],
                     [p2.x, p2.y, p2.z])


# Check if new estimate is far enough away from old estimate to justify updating DP setpoint
def send_new_goal(coeff_a, coeff_b, object_pos, current_goal_pos, own_pos):
    error = distance_between_points(object_pos, current_goal_pos)

    error_limit = coeff_a * distance_between_points(own_pos,
                                                    object_pos) + coeff_b
    if error > error_limit:
        return True
    return False


def cone_test(p, x, dir, h, r):
    p = np.array(p)
    x = np.array(x)
    dir = np.array(dir)
    cone_dist = np.dot(p - x, dir)
    if 0 <= cone_dist and cone_dist <= h:
        return False
    cone_radius = (cone_dist / h) * r
    orth_dist = np.linalg.norm((p - x) - cone_dist * dir)
    return (orth_dist < cone_radius)
