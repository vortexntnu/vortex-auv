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
        ),
        "orientation_final_deg": float(np.degrees(angle[-1])),
        "position_nees_final": float(needs[-1]),
        "position_nees_time_mean": float(needs[moving].mean()),
        "position_nees_fraction_in_single_sample_95_interval": float(
            np.mean((needs[moving] >= 0.2158) & (needs[moving] <= 9.3484))
        ),
        "last_accepted_dvl_time": float(output[-1, 20]),
        "rejected_dvl_including_startup": int(output[-1, 21]),
    }
    return metrics, np.column_stack((output[:, 0], errors, needs, output[:, 20]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--seeds", type=int, default=12)
    parser.add_argument(
        "--suite", choices=("all", "integration", "scenario"), default="all"
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    summary_path = args.output / "summary.json"
    report = json.loads(summary_path.read_text()) if summary_path.exists() else {}

    def case(name, rows, truth):
        result, digest = replay(args.binary, rows)
        metrics, series = evaluate(result, truth)
        metrics["measurement_sha256"] = digest
        report[name] = metrics
        np.savetxt(
            args.output / f"{name}.csv",
            series,
            delimiter=",",
            header="simulation_time,position_error_m,position_nees,last_accepted_dvl_time",
        )
        print(name, json.dumps(metrics), flush=True)
        return result

    if args.suite == "scenario":
        for noise in (False, True):
            rows, truth, _ = measurements(
                "barrel_roll", 75, 42, noise=noise, rate=1000, dvl_rate=8
            )
            name = "barrel_5s_1000imu_8dvl_" + ("noisy" if noise else "noiseless")
            case(name, rows, truth)
        summary_path.write_text(json.dumps(report, indent=2) + "\n")
        return

    if args.suite == "integration":
        for kind in ("rotate", "barrel_roll"):
            for rate in (200, 400, 800):
                rows, truth, _ = measurements(kind, 75, 42, noise=False, rate=rate)
                case(f"{kind}_noiseless_{rate}hz", rows, truth)
        summary_path.write_text(json.dumps(report, indent=2) + "\n")
        return

    rows, truth, _ = measurements("barrel_roll", 170, 42)
    baseline = case("barrel_roll_170s_seed42", rows, truth)
    repeat, _ = replay(args.binary, rows)
    assert np.array_equal(baseline, repeat), (
        "Identical sensor replay changed the estimate"
    )
    poisoned_truth = truth.copy()
    poisoned_truth[:, 1:4] += [100, -50, 25]
    poisoned_metrics, _ = evaluate(repeat, poisoned_truth)
    report["truth_poisoning_offline"] = {
        "estimate_bit_identical": True,
        "position_rmse_with_shifted_reference_m": poisoned_metrics[
            "position_rmse_after_5s_m"
        ],
        "note": "Truth only reaches the separate Python evaluator; ROS boundary tested separately.",
    }
    dropped = rows.copy()
    dropped[:, 7] = 0
    case("barrel_roll_no_dvl", dropped, truth)
    altered = rows.copy()
    altered[altered[:, 0] >= 5, 1] += 0.02
    changed = case("barrel_roll_accel_x_offset_002", altered, truth)
    assert not np.array_equal(baseline, changed)
    for kind in ("stationary", "straight", "turn", "rotate", "barrel_roll"):
        clean, reference, _ = measurements(kind, 75, 42, noise=False)
        case(f"{kind}_noiseless", clean, reference)
    rows, truth, _ = measurements("straight", 75, 42)
    case("straight_nominal", rows, truth)
    scale_error = rows.copy()
    scale_error[:, 8:11] *= 1.01
    case("straight_dvl_scale_plus_1pct", scale_error, truth)
    rows, truth, _ = measurements("barrel_roll", 75, 42, stress=3)
