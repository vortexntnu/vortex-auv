"""Rigid-body conversion and bounded truth interpolation, independent of ROS."""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def body_state(
    position, rotation, velocity, omega, pose_cov, twist_cov, offset, mounting
):
    """Convert IMU-origin world pose/local twist to body, including covariance.

    offset and mounting describe body_P_imu. Pose rotation errors are world axes;
    both twist components are local axes, matching the navigation ROS output.
    """
    body_rotation = rotation @ mounting.T
    arm_world = body_rotation @ offset
    j_pose = np.eye(6)
    j_pose[:3, 3:] = skew(arm_world)
    j_twist = np.zeros((6, 6))
    j_twist[:3, :3] = mounting
    j_twist[3:, 3:] = mounting
    j_twist[:3, 3:] = skew(offset) @ mounting
    twist = j_twist @ np.concatenate((velocity, omega))
    return (
        position - arm_world,
        body_rotation,
        twist[:3],
        twist[3:],
        j_pose @ pose_cov @ j_pose.T,
        j_twist @ twist_cov @ j_twist.T,
    )


def interpolate(a, b, stamp, max_gap=0.02):
    """Interpolate (time, position, quaternion, local velocity, local omega).

    No extrapolation. Velocities interpolate in world axes before returning to
    the interpolated sensor frame. Quaternion interpolation takes the short arc.
    """
    dt = b[0] - a[0]
    if dt <= 0 or dt > max_gap or not a[0] <= stamp <= b[0]:
        return None
    u = (stamp - a[0]) / dt
    rotations = Rotation.from_quat([a[2], b[2]])
    r = Slerp([a[0], b[0]], rotations)(stamp)

    def local(index):
        return r.inv().apply(
            (1 - u) * rotations[0].apply(a[index]) + u * rotations[1].apply(b[index])
        )

    return stamp, (1 - u) * a[1] + u * b[1], r.as_quat(), local(3), local(4)


def command_allowed(mode, killed, command_age, estimate_age, joy_age):
    """ROS OperationMode values: manual=2, reference=3; autonomous out of scope."""
    return (
        not killed
        and 0 <= command_age <= 0.2
        and 0 <= joy_age <= 0.5
        and (mode == 2 or (mode == 3 and 0 <= estimate_age <= 0.2))
    )


def bottom_track_valid(altitude, velocity):
    """Installed Stonefish bridge reports altitude=-1 for water-only/no ping."""
    return bool(np.isfinite(altitude) and altitude > 0 and np.isfinite(velocity).all())


def bottom_track_tilt_valid(quaternion, max_tilt_deg):
    """Gate the downward DVL axis against world down, including combined tilt.

    Nautilus's IMU and DVL differ only by yaw, so their positive z axes coincide.
    Quaternion is the native simulator attitude, never an estimated attitude.
    """
    q = np.asarray(quaternion, dtype=float)
    norm_squared = float(q @ q)
    if not np.isfinite(q).all() or norm_squared < 1e-12:
        return False
    cosine = 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2) / norm_squared
    return bool(cosine >= np.cos(np.deg2rad(max_tilt_deg)) - 1e-12)
