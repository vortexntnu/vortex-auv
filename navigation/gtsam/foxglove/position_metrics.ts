// Paste into a Foxglove User Script with the simulation connected.
import { Input } from "./types.ts";

// Types begin (types.ts is supplied by Foxglove).
type Stamp = { sec: number; nsec?: number; nanosec?: number };
type Odom = {
  header: { stamp: Stamp; frame_id: string };
  child_frame_id: string;
  pose: {
    pose: { position: { x: number; y: number; z: number } };
    covariance: ArrayLike<number>;
  };
};
type Output = {
  header: { stamp: { sec: number; nsec: number }; frame_id: string };
  matched_samples: number;
  position_error_m: number;
  rmse_position_m: number;
  rmse_x_m: number;
  rmse_y_m: number;
  rmse_z_m: number;
  position_nees: number;
  position_nees_per_dof: number;
  nees_expected: number;
  nees_lower_95: number;
  nees_upper_95: number;
};
// Types end.

export const inputs = ["/nautilus/gtsam/odom", "/nautilus/gtsam/truth"];
export const output = "/foxglove_script/gtsam_position_metrics";

const estimates = new Map<string, Odom>();
const truths = new Map<string, Odom>();
const lastTimes = new Map<string, number>();
let count = 0;
let squaredErrors = [0, 0, 0];

function stamp(message: Odom) {
  const s = message.header.stamp;
  const nsec = s.nsec !== undefined ? s.nsec : s.nanosec;
  return { sec: s.sec, nsec: nsec !== undefined ? nsec : 0 };
}

// Compute ||L^-1 e||^2 for P = L L^T, retaining cross-covariances.
function positionNees(error: number[], covariance: ArrayLike<number>) {
  const lower = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; ++i) {
    for (let j = 0; j <= i; ++j) {
      const a = covariance[6 * i + j];
      const b = covariance[6 * j + i];
      if (!Number.isFinite(a) || !Number.isFinite(b) ||
          Math.abs(a - b) > 1e-8 * Math.max(1, Math.abs(a), Math.abs(b))) {
        return undefined;
      }
      let value = (a + b) / 2;
      for (let k = 0; k < j; ++k) value -= lower[i][k] * lower[j][k];
      if (i === j) {
        if (!(value > 0)) return undefined;
        lower[i][j] = Math.sqrt(value);
      } else {
        lower[i][j] = value / lower[j][j];
      }
    }
  }
  const whitened = [0, 0, 0];
  for (let i = 0; i < 3; ++i) {
    let value = error[i];
    for (let j = 0; j < i; ++j) value -= lower[i][j] * whitened[j];
    whitened[i] = value / lower[i][i];
  }
  return whitened.reduce((sum, value) => sum + value * value, 0);
}

export default function script(
  event: Input<"/nautilus/gtsam/odom"> | Input<"/nautilus/gtsam/truth">,
): Output | undefined {
  const message = event.message;
  const s = stamp(message);
  const time = s.sec + s.nsec * 1e-9;
  const previous = lastTimes.get(event.topic);
  // Each input must be ordered. A backward jump starts a new evaluation run.
  if (previous !== undefined && time < previous) {
    estimates.clear();
    truths.clear();
    lastTimes.clear();
    count = 0;
    squaredErrors = [0, 0, 0];
  } else if (previous === time) {
    return undefined;
  }
  lastTimes.set(event.topic, time);
  const key = `${s.sec}:${s.nsec}`;
  const cache = event.topic === inputs[0] ? estimates : truths;
  cache.set(key, message);
  // Bounded buffers accommodate either arrival order and estimator latency.
  while (cache.size > 2000) {
    const oldest = cache.keys().next().value;
    if (oldest === undefined) break;
    cache.delete(oldest);
  }
  const estimate = estimates.get(key);
  const truth = truths.get(key);
  if (!estimate || !truth) return undefined;
  estimates.delete(key);
  truths.delete(key);
  if (estimate.header.frame_id !== truth.header.frame_id ||
      estimate.child_frame_id !== truth.child_frame_id) return undefined;
  const p = estimate.pose.pose.position;
  const q = truth.pose.pose.position;
  const error = [p.x - q.x, p.y - q.y, p.z - q.z];
  if (!error.every(Number.isFinite)) return undefined;
  const nees = positionNees(error, estimate.pose.covariance);
  if (nees === undefined || !Number.isFinite(nees)) return undefined;
  ++count;
  for (let i = 0; i < 3; ++i) squaredErrors[i] += error[i] * error[i];
  return {
    header: { stamp: s, frame_id: estimate.header.frame_id },
    matched_samples: count,
    position_error_m: Math.hypot(...error),
    rmse_position_m: Math.sqrt(squaredErrors.reduce((a, b) => a + b, 0) / count),
    rmse_x_m: Math.sqrt(squaredErrors[0] / count),
    rmse_y_m: Math.sqrt(squaredErrors[1] / count),
    rmse_z_m: Math.sqrt(squaredErrors[2] / count),
    position_nees: nees,
    position_nees_per_dof: nees / 3,
    nees_expected: 3,
    nees_lower_95: 0.2158,
    nees_upper_95: 9.3484,
  };
}
