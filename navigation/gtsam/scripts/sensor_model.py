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
    u = time / duration
    return (
        angle * (10 * u**3 - 15 * u**4 + 6 * u**5),
        angle / duration * 30 * u**2 * (1 - u) ** 2,
        angle / duration**2 * 60 * u * (1 - u) * (1 - 2 * u),
    )


def finite_forward_motion(time, distance, speed=0.3):
    """Translate a fixed distance, using two-second cosine acceleration/braking."""
    acceleration_time = 2.0
    duration = distance / speed + acceleration_time
    if time >= duration:
        return distance, 0.0, 0.0
    start = ramp(time, speed, acceleration_time)
    stop = ramp(time - (duration - acceleration_time), speed, acceleration_time)
    return tuple(a - b for a, b in zip(start, stop, strict=True))


def square_barrel_roll(time):
    """A 3 m square with stopped yaw turns, then one forward roll and final rest.

    Time is relative to the end of stationary alignment. Each square leg takes
    12 s, followed by a 1 s stop, a 4 s yaw turn and a 1 s pause. The fourth yaw
    restores the original heading at the starting point. The final forward
    segment accelerates for 2 s, rolls for 5 s and brakes for 2 s.
    """
    corners = ((0, 0, 0), (3, 0, 0), (3, 3, 0), (0, 3, 0))
    directions = ((1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0))
    if time < 72:
        leg = min(3, max(0, int(time // 18)))
        local_time = time - 18 * leg
        distance, speed, acceleration = finite_forward_motion(local_time, 3.0)
        turn, rate, alpha = smooth_rotation(local_time - 13, math.pi / 2, 4.0)
        direction = np.array(directions[leg], dtype=float)
        return (
            np.array(corners[leg], dtype=float) + distance * direction,
            speed * direction,
            acceleration * direction,
            rotation_z(leg * math.pi / 2 + turn),
            np.array([0.0, 0.0, rate]),
            np.array([0.0, 0.0, alpha]),
        )
    local_time = time - 72
    distance, speed, acceleration = finite_forward_motion(local_time, 2.1)
    roll, rate, alpha = smooth_rotation(local_time - 2, 2 * math.pi, 5.0)
    c, s = math.cos(roll), math.sin(roll)
    return (
        np.array([distance, 0.0, 0.0]),
        np.array([speed, 0.0, 0.0]),
        np.array([acceleration, 0.0, 0.0]),
        np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
        np.array([rate, 0.0, 0.0]),
        np.array([alpha, 0.0, 0.0]),
    )


def trajectory(time, kind="turn", settle=5.0):
    """Return body-origin world position/velocity/acceleration and body rotation/rates."""
    if kind == "square_barrel_roll":
        return square_barrel_roll(time - settle)
    if kind not in ("stationary", "straight", "turn", "rotate", "barrel_roll"):
        raise ValueError(f"Unknown trajectory: {kind}")
    p, v, a = ramp(time - settle, 0.3 if kind != "stationary" else 0.0)
    yaw, omega, alpha = ramp(time - settle, 0.08 if kind == "turn" else 0.0)
    attitude = (
        rotating_attitude(time - settle)
        if kind == "rotate"
        else (rotation_z(yaw), np.array([0.0, 0.0, omega]), np.array([0.0, 0.0, alpha]))
    )
    if kind == "barrel_roll":
        roll, rate, acceleration = ramp(time - settle, 2 * math.pi / 5.0)
        c, s = math.cos(roll), math.sin(roll)
        attitude = (
            np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            np.array([rate, 0.0, 0.0]),
            np.array([acceleration, 0.0, 0.0]),
        )
    return (
        np.array([p, 0.0, 0.0]),
        np.array([v, 0.0, 0.0]),
        np.array([a, 0.0, 0.0]),
        *attitude,
    )


def bottom_lock_available(rotation, max_tilt_deg=30.0):
    """Simplified flat-seabed visibility from DVL +Z boresight tilt to world down.

    The fixed DVL mount differs from the body by yaw only, so both +Z axes
    coincide. The limit is a simulation assumption, not a Nucleus specification.
    """
    if not math.isfinite(max_tilt_deg) or not 0 < max_tilt_deg < 90:
        raise ValueError("DVL maximum tilt must be between 0 and 90 degrees")
    return bool(rotation[2, 2] >= math.cos(math.radians(max_tilt_deg)))


def ideal_measurements(state, gravity=9.81):
    """Include both tangential and centripetal acceleration of the fixed IMU."""
    _, velocity, acceleration, rotation, omega, alpha = state
    specific_force = rotation.T @ (acceleration - np.array([0.0, 0.0, gravity]))
    specific_force += np.cross(alpha, IMU_OFFSET) + np.cross(
        omega, np.cross(omega, IMU_OFFSET)
    )
    dvl = rotation_z(DVL_YAW).T @ (rotation.T @ velocity + np.cross(omega, DVL_OFFSET))
    return specific_force, omega.copy(), dvl


class SensorModel:
    """White rate noise plus independent continuous bias random walks.

    Drift densities and initial offsets below are modeling assumptions. They are
    deliberately separate from the datasheet's Allan bias-instability minima.
    """

    def __init__(
        self,
        seed=42,
        profile="stim300_10g_provisional",
        noise=True,
        stress_scale=1.0,
        accel_bias=(0.003, -0.002, 0.001),
        gyro_bias=(2e-5, -1e-5, 1e-5),
        accel_bias_rw=1e-5,
        gyro_bias_rw=1e-7,
    ):
        if profile not in ACCEL_DENSITIES:
            raise ValueError(f"Unknown IMU profile: {profile}")
        if not math.isfinite(stress_scale) or stress_scale <= 0:
            raise ValueError("stress_scale must be positive")
        self.rng = np.random.default_rng(seed)
        self.accel_density = ACCEL_DENSITIES[profile] * stress_scale
        self.gyro_density = GYRO_DENSITY * stress_scale
        self.accel_bias = np.array(accel_bias, dtype=float) if noise else np.zeros(3)
        self.gyro_bias = np.array(gyro_bias, dtype=float) if noise else np.zeros(3)
        self.accel_bias_rw = accel_bias_rw * stress_scale
        self.gyro_bias_rw = gyro_bias_rw * stress_scale
        self.noise = noise

    def imu(self, acceleration, omega, dt):
        """Sample noise with density/sqrt(dt), and bias increments with density*sqrt(dt)."""
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        if not self.noise:
            return acceleration.copy(), omega.copy()
        self.accel_bias += self.rng.normal(size=3) * self.accel_bias_rw * math.sqrt(dt)
        self.gyro_bias += self.rng.normal(size=3) * self.gyro_bias_rw * math.sqrt(dt)
        return (
            acceleration
            + self.accel_bias
            + self.rng.normal(size=3) * self.accel_density / math.sqrt(dt),
            omega
            + self.gyro_bias
            + self.rng.normal(size=3) * self.gyro_density / math.sqrt(dt),
        )

    def dvl(self, velocity, sigma=0.005):
        """Sample independent 0.5 cm/s single-ping velocity noise by default."""
        return velocity + (self.rng.normal(size=3) * sigma if self.noise else 0)
