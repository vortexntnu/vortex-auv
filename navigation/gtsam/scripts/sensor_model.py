"""Deterministic sensor physics; no ROS dependency or estimator truth feedback."""

import math

import numpy as np

IMU_OFFSET = np.array([-0.100, -0.001, 0.085])
DVL_OFFSET = np.array([-0.130, 0.003, 0.2414])
# Match the existing URDF, including its rounded pi value.
DVL_YAW = 3.14159
GYRO_DENSITY = math.radians(0.15) / 60.0
ACCEL_DENSITIES = {
    "stim300_10g_provisional": 0.07 / 60.0,
    "stim300_10g": 0.07 / 60.0,
    "stim300_30g": 0.21 / 60.0,
}


def rotation_z(angle):
    """Return a right-handed rotation about z."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quaternion_from_rotation(rotation):
    """Return a normalized xyzw quaternion, including rotations near 180 degrees."""
    q = np.zeros(4)
    trace = np.trace(rotation)
    if trace > 0:
        q[3] = math.sqrt(1 + trace) / 2
        q[:3] = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        ) / (4 * q[3])
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        q[i] = math.sqrt(1 + rotation[i, i] - rotation[j, j] - rotation[k, k]) / 2
        q[j] = (rotation[j, i] + rotation[i, j]) / (4 * q[i])
        q[k] = (rotation[k, i] + rotation[i, k]) / (4 * q[i])
        q[3] = (rotation[k, j] - rotation[j, k]) / (4 * q[i])
    return q / np.linalg.norm(q)


def ramp(time, target, duration=4.0):
    """Return integral, value and derivative of a smooth rate ramp."""
    if time <= 0:
        return 0.0, 0.0, 0.0
    if time >= duration:
        return target * (time - duration / 2), target, 0.0
    phase = math.pi * time / duration
    return (
        target / 2 * (time - duration / math.pi * math.sin(phase)),
        target / 2 * (1 - math.cos(phase)),
        target / 2 * math.pi / duration * math.sin(phase),
    )


def rotating_attitude(time):
    """Continuous Rz(yaw) Ry(pitch) Rx(roll) revolutions, with body-axis rates."""
    roll, roll_rate, roll_acceleration = ramp(time, math.radians(12))
    pitch, pitch_rate, pitch_acceleration = ramp(time, math.radians(9))
    yaw, yaw_rate, yaw_acceleration = ramp(time, math.radians(6))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    roll_axis = np.array([1.0, 0.0, 0.0])
    pitch_axis = rx.T @ np.array([0.0, 1.0, 0.0])
    yaw_axis = rx.T @ ry.T @ np.array([0.0, 0.0, 1.0])
    wr, wp, wy = roll_rate * roll_axis, pitch_rate * pitch_axis, yaw_rate * yaw_axis
    # The pitch/yaw axes themselves move in body coordinates as attitude changes.
    alpha = (
        roll_acceleration * roll_axis
        + pitch_acceleration * pitch_axis
        + yaw_acceleration * yaw_axis
    )
    alpha -= np.cross(wr, wp + wy) + np.cross(wp, wy)
    return rotation_z(yaw) @ ry @ rx, wr + wp + wy, alpha


def smooth_rotation(time, angle, duration):
    """Finite rotation with zero angular velocity and acceleration at both ends."""
    if time <= 0:
        return 0.0, 0.0, 0.0
    if time >= duration:
        return angle, 0.0, 0.0
