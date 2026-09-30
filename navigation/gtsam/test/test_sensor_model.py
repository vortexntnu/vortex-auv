"""Check sensor physics and continuous-density discretization independently of ROS."""

import importlib.util
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest

MODEL_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sensor_model.py"
SPEC = importlib.util.spec_from_file_location("sensor_model", MODEL_PATH)
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


def test_simulation_mounts_match_existing_nautilus():
    description = (
        pathlib.Path(__file__).resolve().parents[3]
        / "auv_setup"
        / "description"
        / "nautilus.urdf.xacro"
    )
    robot = ET.parse(description).getroot()  # noqa: S314 - trusted, checked-in fixture
    for joint_name, offset, yaw in (
        ("base_to_imu", MODEL.IMU_OFFSET, 0.0),
        ("base_to_dvl", MODEL.DVL_OFFSET, MODEL.DVL_YAW),
    ):
        origin = robot.find(f"joint[@name='{joint_name}']/origin")
        np.testing.assert_allclose(np.fromstring(origin.attrib["xyz"], sep=" "), offset)
        np.testing.assert_allclose(
            np.fromstring(origin.attrib["rpy"], sep=" "), [0, 0, yaw]
        )


def test_stationary_specific_force_and_seed():
    a, w, dvl = MODEL.ideal_measurements(MODEL.trajectory(0))
    np.testing.assert_allclose(a, [0, 0, -9.81])
    np.testing.assert_allclose(w, 0)
    np.testing.assert_allclose(dvl, 0)
    first, second = MODEL.SensorModel(seed=7), MODEL.SensorModel(seed=7)
    for _ in range(100):
        np.testing.assert_array_equal(first.imu(a, w, 0.01), second.imu(a, w, 0.01))


def test_noise_density_and_range_profiles():
    variances = []
    for profile in ("stim300_10g", "stim300_30g"):
        model = MODEL.SensorModel(
            profile=profile,
            accel_bias=(0, 0, 0),
            gyro_bias=(0, 0, 0),
            accel_bias_rw=0,
            gyro_bias_rw=0,
        )
        samples = np.array(
            [model.imu(np.zeros(3), np.zeros(3), 0.01)[0] for _ in range(10000)]
        )
        expected = MODEL.ACCEL_DENSITIES[profile] ** 2 / 0.01
        assert abs(samples.var() / expected - 1) < 0.04
        variances.append(samples.var())
    np.testing.assert_allclose(variances[1] / variances[0], 9.0, rtol=1e-10)


def test_rotating_offsets_include_tangential_acceleration():
    state = MODEL.trajectory(7, "turn")
    a, w, dvl = MODEL.ideal_measurements(state)
    _, velocity, acceleration, rotation, omega, alpha = state
    imu_acceleration = rotation.T @ (acceleration - [0, 0, 9.81])
    imu_acceleration += np.cross(alpha, MODEL.IMU_OFFSET) + np.cross(
        w, np.cross(w, MODEL.IMU_OFFSET)
    )
    np.testing.assert_allclose(a, imu_acceleration)
    imu_velocity = rotation.T @ velocity + np.cross(omega, MODEL.IMU_OFFSET)
    relative_arm = MODEL.DVL_OFFSET - MODEL.IMU_OFFSET
    np.testing.assert_allclose(
        MODEL.rotation_z(MODEL.DVL_YAW) @ dvl,
        imu_velocity + np.cross(omega, relative_arm),
    )


def test_three_axis_trajectory_rates_match_rotation_derivatives():
    step = 1e-4
    for time in (5.5, 7.0, 10.0, 17.0, 22.0, 45.0, 47.0, 67.0, 80.0, 127.0):
        state = MODEL.trajectory(time, "rotate")
        before = MODEL.trajectory(time - step, "rotate")
        after = MODEL.trajectory(time + step, "rotate")
        rotation, omega, alpha = state[3:]
        skew = rotation.T @ ((after[3] - before[3]) / (2 * step))
        np.testing.assert_allclose(
            omega, [skew[2, 1], skew[0, 2], skew[1, 0]], atol=1e-9
