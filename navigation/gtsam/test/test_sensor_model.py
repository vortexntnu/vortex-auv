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
        )
        np.testing.assert_allclose(
            alpha, (after[4] - before[4]) / (2 * step), atol=1e-9
        )
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-14)
    assert np.all(np.abs(MODEL.trajectory(7, "rotate")[4]) > 0.005)


@pytest.mark.parametrize("kind", ["rotate", "barrel_roll", "square_barrel_roll"])
def test_rotating_sensor_measurements_match_sensor_position_derivatives(kind):
    step = 1e-3
    # Test finite differences inside segments; endpoint continuity is checked
    # separately because jerk can change there, reducing difference accuracy.
    for time in (5.5, 7.3, 10.0, 20.0, 45.3, 80.0):
        state = MODEL.trajectory(time, kind)
        before = MODEL.trajectory(time - step, kind)
        after = MODEL.trajectory(time + step, kind)
        force, _, dvl = MODEL.ideal_measurements(state)
        p_imu = state[0] + state[3] @ MODEL.IMU_OFFSET
        p_imu_before = before[0] + before[3] @ MODEL.IMU_OFFSET
        p_imu_after = after[0] + after[3] @ MODEL.IMU_OFFSET
        world_acceleration = (p_imu_after - 2 * p_imu + p_imu_before) / step**2
        np.testing.assert_allclose(
            state[3] @ force + [0, 0, 9.81], world_acceleration, atol=2e-7
        )
        # Velocity differences can use a finer step without the cancellation
        # inherent in the second position derivative above.
        velocity_step = step / 10
        before_v = MODEL.trajectory(time - velocity_step, kind)
        after_v = MODEL.trajectory(time + velocity_step, kind)
        p_dvl_before = before_v[0] + before_v[3] @ MODEL.DVL_OFFSET
        p_dvl_after = after_v[0] + after_v[3] @ MODEL.DVL_OFFSET
        world_velocity = (p_dvl_after - p_dvl_before) / (2 * velocity_step)
        np.testing.assert_allclose(
            state[3] @ MODEL.rotation_z(MODEL.DVL_YAW) @ dvl, world_velocity, atol=5e-8
        )


@pytest.mark.parametrize("kind", ["rotate", "barrel_roll", "square_barrel_roll"])
def test_rotating_trajectory_preserves_stationary_alignment(kind):
    for time in (0.0, 2.0, 4.999, 5.0):
        for actual, expected in zip(
            MODEL.trajectory(time, kind),
            MODEL.trajectory(time, "stationary"),
            strict=True,
        ):
            np.testing.assert_allclose(actual, expected, atol=1e-14)


def test_truth_quaternion_matches_rotation_including_half_turns():
    rotations = [MODEL.trajectory(t, "rotate")[3] for t in (0, 7, 10, 45, 80)]
    rotations += [np.diag([1, -1, -1]), np.diag([-1, 1, -1]), np.diag([-1, -1, 1])]
    for rotation in rotations:
        x, y, z, w = MODEL.quaternion_from_rotation(rotation)
        reconstructed = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )
        np.testing.assert_allclose(reconstructed, rotation, atol=1e-14)


def test_continuous_rotation_passes_full_revolutions():
    # After a 5 s alignment and 4 s ramp, the integrated steady-rate time is t-7.
    # Distinct full-turn counts (4 roll, 3 pitch, 2 yaw) coincide after 120 s.
    np.testing.assert_allclose(
        MODEL.trajectory(127, "rotate")[3], np.eye(3), atol=1e-13
    )
    assert np.linalg.norm(MODEL.trajectory(127, "rotate")[4]) > 0.2
    # Pitch passes through 90 and 180 degrees instead of reversing at a small tilt.
    rotation_quarter = MODEL.trajectory(17, "rotate")[3]
    rotation_half = MODEL.trajectory(27, "rotate")[3]
    np.testing.assert_allclose(rotation_quarter[2, 0], -1, atol=1e-13)
    np.testing.assert_allclose(rotation_half[2, 0], 0, atol=1e-13)
    np.testing.assert_allclose(rotation_half[2, 2], 0.5, atol=1e-13)


def test_barrel_roll_keeps_forward_axis_aligned_and_completes_revolutions():
    for time in (0, 7, 10, 22, 37, 67):
        position, velocity, _, rotation, omega, _ = MODEL.trajectory(
            time, "barrel_roll"
        )
        np.testing.assert_allclose(rotation @ [1, 0, 0], [1, 0, 0], atol=1e-14)
        np.testing.assert_allclose(position[1:], 0)
        np.testing.assert_allclose(velocity[1:], 0)
        np.testing.assert_allclose(omega[1:], 0)
    np.testing.assert_allclose(
        MODEL.trajectory(9.5, "barrel_roll")[3], np.diag([1, -1, -1]), atol=1e-14
    )
    np.testing.assert_allclose(
        MODEL.trajectory(12, "barrel_roll")[3], np.eye(3), atol=1e-14
    )
    np.testing.assert_allclose(
        MODEL.trajectory(17, "barrel_roll")[3], np.eye(3), atol=1e-14
    )
    for time in (9.0, 11.3, 25.0):
        state = MODEL.trajectory(time, "barrel_roll")
        np.testing.assert_allclose(state[4], [2 * np.pi / 5, 0, 0], atol=1e-14)
        np.testing.assert_allclose(
            MODEL.trajectory(time + 5, "barrel_roll")[3], state[3], atol=1e-14
        )


def test_bottom_lock_lost_when_tilted_and_restored_upright():
    for time, expected in (
        (5, True),
        (7.0, True),
        (7.2, False),
        (9.5, False),
        (11.5, False),
        (11.625, True),
        (12, True),
        (12.5, False),
    ):
        assert (
            MODEL.bottom_lock_available(MODEL.trajectory(time, "barrel_roll")[3])
            == expected
        )
    assert MODEL.bottom_lock_available(MODEL.trajectory(11.25, "barrel_roll")[3], 65)
    for limit in (0, 90, -1, float("nan")):
        with pytest.raises(ValueError, match="tilt"):
            MODEL.bottom_lock_available(np.eye(3), limit)


def test_square_returns_to_start_aligns_rolls_once_and_stops():
    for time, position, yaw in (
        (17, [3, 0, 0], 0),
        (35, [3, 3, 0], np.pi / 2),
        (53, [0, 3, 0], np.pi),
        (71, [0, 0, 0], 3 * np.pi / 2),
        (77, [0, 0, 0], 0),
        (86, [2.1, 0, 0], 0),
        (150, [2.1, 0, 0], 0),
    ):
        p, v, a, rotation, omega, alpha = MODEL.trajectory(time, "square_barrel_roll")
        np.testing.assert_allclose(p, position, atol=1e-13)
        np.testing.assert_allclose(rotation, MODEL.rotation_z(yaw), atol=1e-13)
        for derivative in (v, a, omega, alpha):
            np.testing.assert_allclose(derivative, 0, atol=1e-13)
    # The single roll starts upright at 79 s, passes inverted at 81.5 s,
    # and ends upright at 84 s while still moving forward at cruising speed.
    for time, rotation in (
        (79, np.eye(3)),
        (81.5, np.diag([1, -1, -1])),
        (84, np.eye(3)),
    ):
        state = MODEL.trajectory(time, "square_barrel_roll")
        np.testing.assert_allclose(state[1], [0.3, 0, 0], atol=1e-13)
        np.testing.assert_allclose(state[3], rotation, atol=1e-13)
    assert not MODEL.bottom_lock_available(
        MODEL.trajectory(81.5, "square_barrel_roll")[3]
    )


def test_square_boundary_continuity_and_forward_only_translation():
    boundaries = {0.0, 5.0, 77.0, 79.0, 84.0, 86.0}
    for leg in range(4):
        boundaries.update(
            5 + 18 * leg + offset for offset in (0, 2, 10, 12, 13, 17, 18)
        )
    for time in boundaries:
        before = MODEL.trajectory(time - 1e-6, "square_barrel_roll")
        after = MODEL.trajectory(time + 1e-6, "square_barrel_roll")
        for left, right in zip(before, after, strict=True):
            np.testing.assert_allclose(left, right, atol=4e-6)
    for time in np.linspace(0, 100, 1001):
        p, v, _, rotation, omega, _ = MODEL.trajectory(time, "square_barrel_roll")
        local_velocity = rotation.T @ v
        np.testing.assert_allclose(local_velocity[1:], 0, atol=1e-13)
        assert -1e-13 <= local_velocity[0] <= 0.3 + 1e-13
        assert abs(p[2]) < 1e-13
        if time < 79:
            assert MODEL.bottom_lock_available(rotation)
            if np.linalg.norm(omega) > 1e-10:
                np.testing.assert_allclose(v, 0, atol=1e-13)


def test_square_rates_and_accelerations_match_pose_derivatives():
