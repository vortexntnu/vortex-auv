"""Offline audit of the production estimator with generated sensor-only input.

Run explicitly; this is a sensitivity experiment, not a hardware acceptance test.
The replay executable receives no truth, biases, seed, lock flag, or trajectory.
"""

import argparse
import hashlib
import importlib.util
import io
import json
import pathlib
import subprocess

import numpy as np

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "sensor_model", PACKAGE / "scripts" / "sensor_model.py"
)
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


def measurements(kind, duration, seed, noise=True, stress=1.0, rate=200, dvl_rate=5):
    """Keep all potential DVL pings so ablations reuse identical IMU samples."""
    model = MODEL.SensorModel(seed=seed, noise=noise, stress_scale=stress)
    rows, truths, locks = [], [], []
    for index in range(round(duration * rate) + 1):
        time = index / rate
        state = MODEL.trajectory(time, kind)
        force, gyro, dvl = MODEL.ideal_measurements(state)
        force, gyro = model.imu(force, gyro, 1.0 / rate)
        lock = MODEL.bottom_lock_available(state[3])
        lock = lock and not (kind != "square_barrel_roll" and 20 <= time < 25)
        # Match the production random draw schedule exactly: DVL noise is drawn
        # only on available pings. Removing pings later must not change the IMU.
        ping = index % round(rate / dvl_rate) == 0 and lock
        measured_dvl = model.dvl(dvl) if ping else dvl
        rows.append([time, *force, *gyro, int(ping), *measured_dvl])
        p, v, _, rotation, omega, _ = state
        truths.append(
            [
                time,
                *(p + rotation @ MODEL.IMU_OFFSET - MODEL.IMU_OFFSET),
                *MODEL.quaternion_from_rotation(rotation),
                *(v + rotation @ np.cross(omega, MODEL.IMU_OFFSET)),
            ]
        )
        locks.append(lock)
    return np.array(rows), np.array(truths), np.array(locks)


def replay(binary, rows):
    stream = io.StringIO()
    np.savetxt(stream, rows, fmt="%.17g")
    data = stream.getvalue()
    result = subprocess.run(  # noqa: S603 - explicit local audit executable
        [str(binary)], input=data, text=True, capture_output=True, check=True
    )
    return np.loadtxt(io.StringIO(result.stdout)), hashlib.sha256(
        data.encode()
    ).hexdigest()


def evaluate(output, truths):
    indices = np.rint(output[:, 0] / (truths[1, 0] - truths[0, 0])).astype(int)
    truth = truths[indices]
    np.testing.assert_allclose(output[:, 0], truth[:, 0], atol=1e-10, rtol=0)
    error = output[:, 1:4] - truth[:, 1:4]
    covariance = output[:, 11:20].reshape(-1, 3, 3)
    assert np.all(np.linalg.eigvalsh(covariance) > 0)
    whitened = np.linalg.solve(covariance, error[..., None])[..., 0]
    needs = np.einsum("ni,ni->n", error, whitened)
    angle = 2 * np.arccos(
        np.clip(np.abs(np.sum(output[:, 4:8] * truth[:, 4:8], axis=1)), 0, 1)
    )
    moving = output[:, 0] >= 5
    errors = np.linalg.norm(error, axis=1)
    metrics = {
        "samples": len(output),
        "position_rmse_after_5s_m": float(np.sqrt(np.mean(errors[moving] ** 2))),
        "position_final_m": float(errors[-1]),
        "position_max_m": float(errors.max()),
        "velocity_rmse_after_5s_mps": float(
            np.sqrt(
                np.mean(
                    np.sum((output[moving, 8:11] - truth[moving, 8:11]) ** 2, axis=1)
                )
            )
