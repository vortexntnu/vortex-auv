import math

from geometry_msgs.msg import Pose
from vortex_msgs.msg import Waypoint


def make_pose(x, y, z, yaw=0.0) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    half = yaw * 0.5
    pose.orientation.z = math.sin(half)
    pose.orientation.w = math.cos(half)
    return pose


def make_waypoint(x, y, z, yaw=0.0, mode=Waypoint.FULL_POSE) -> Waypoint:
    wp = Waypoint()
    wp.pose = make_pose(x, y, z, yaw)
    wp.mode = mode
    return wp


def pose_distance(p1: Pose, p2: Pose):
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    dz = p1.position.z - p2.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)
