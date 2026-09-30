"""Independent rigid-body, covariance, interpolation, and command guard checks."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

SPEC = importlib.util.spec_from_file_location(
    'stonefish_math', Path(__file__).resolve().parents[1] / 'scripts/stonefish_math.py'
)
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def test_rotating_body_origin_is_stationary():
    r = Rotation.from_euler('xyz', [0.4, -0.6, 1.2]).as_matrix()
    mounting = Rotation.from_euler('z', 0.7).as_matrix()
    arm = np.array([0.2, -0.1, 0.4])
    omega = np.array([0.3, -0.4, 0.8])
    state = M.body_state(
        r @ arm,
        r @ mounting,
        mounting.T @ np.cross(omega, arm),
        mounting.T @ omega,
        np.eye(6),
        np.eye(6),
        arm,
        mounting,
    )
    np.testing.assert_allclose(state[0], 0, atol=1e-14)
    np.testing.assert_allclose(state[1], r, atol=1e-14)
    np.testing.assert_allclose(state[2], 0, atol=1e-14)
    np.testing.assert_allclose(state[3], omega, atol=1e-14)


def test_covariance_matches_independent_perturbations():
    rng = np.random.default_rng(12)
    arm = np.array([0.2, -0.1, 0.4])
    r = Rotation.from_euler('xyz', [0.4, -0.2, 0.8]).as_matrix()
    errors = rng.normal(size=(70000, 6)) * 1e-4
    p_errors = (
        -(Rotation.from_rotvec(errors[:, 3:]).apply(r @ arm) - r @ arm) + errors[:, :3]
    )
    out = M.body_state(
        np.zeros(3),
        r,
        np.zeros(3),
        np.zeros(3),
        np.eye(6) * 1e-8,
        np.eye(6),
        arm,
        np.eye(3),
    )
    empirical = np.cov(np.c_[p_errors, errors[:, 3:]].T)
    np.testing.assert_allclose(empirical, out[4], atol=1.8e-10)
    assert np.linalg.eigvalsh(out[5]).min() > 0


def test_interpolation_and_no_extrapolation():
    a = (10.0, np.zeros(3), [0, 0, 0, 1], np.array([1, 0, 0]), np.zeros(3))
    b = (10.01, np.ones(3), [0, 0, 1, 0], np.array([-1, 0, 0]), np.zeros(3))
    mid = M.interpolate(a, b, 10.005)
    np.testing.assert_allclose(mid[1], 0.5, atol=1e-12)
    np.testing.assert_allclose(
        Rotation.from_quat(mid[2]).apply(mid[3]), [1, 0, 0], atol=1e-12
    )
    assert M.interpolate(a, b, 9.0) is None
    assert M.interpolate(a, (11.0, *b[1:]), 10.5) is None


@pytest.mark.parametrize(
    ('mode', 'killed', 'cmd', 'est', 'joy', 'expected'),
    [
        (2, False, 0.01, 999, 0.01, True),
        (3, False, 0.01, 0.01, 0.01, True),
        (3, False, 0.01, 0.3, 0.01, False),
        (3, False, 0.01, -0.1, 0.01, False),
        (3, True, 0.01, 0.01, 0.01, False),
        (2, False, 0.3, 0.01, 0.01, False),
        (3, False, 0.01, 0.01, 0.6, False),
        (1, False, 0.01, 0.01, 0.01, False),
    ],
)
def test_command_guard(mode, killed, cmd, est, joy, expected):
    assert M.command_allowed(mode, killed, cmd, est, joy) == expected


@pytest.mark.parametrize(
    ('altitude', 'velocity', 'expected'),
    [
        (1.0, [0.0, 0.0, 0.0], True),
        (-1.0, [0.0, 0.0, 0.0], False),
        (0.0, [1.0, 2.0, 3.0], False),
        (float('nan'), [0.0, 0.0, 0.0], False),
        (1.0, [float('inf'), 0.0, 0.0], False),
    ],
)
def test_native_dvl_validity(altitude, velocity, expected):
    assert M.bottom_track_valid(altitude, velocity) == expected


@pytest.mark.parametrize(
    ('angles', 'expected'),
    [
        ([0.0, 0.0, 180.0], True),
        ([24.9, 0.0, 0.0], True),
        ([25.0, 0.0, 0.0], True),
        ([25.1, 0.0, 0.0], False),
        ([-25.1, 0.0, 0.0], False),
        ([0.0, 25.1, 70.0], False),
        ([20.0, 20.0, 0.0], False),
        ([90.0, 0.0, 0.0], False),
        ([180.0, 0.0, 0.0], False),
        ([360.0, 0.0, 0.0], True),
    ],
)
def test_bottom_track_tilt(angles, expected):
    q = Rotation.from_euler('xyz', angles, degrees=True).as_quat()
    assert M.bottom_track_tilt_valid(q, 25.0) == expected
    assert M.bottom_track_tilt_valid(-2 * q, 25.0) == expected


def test_invalid_attitude_has_no_lock():
    assert not M.bottom_track_tilt_valid([0.0, 0.0, 0.0, 0.0], 25.0)
    assert not M.bottom_track_tilt_valid([float('nan'), 0.0, 0.0, 1.0], 25.0)
