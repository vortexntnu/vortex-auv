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
