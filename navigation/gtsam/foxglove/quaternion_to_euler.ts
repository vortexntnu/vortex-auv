// Paste into a Foxglove User Script. These are display angles, not an estimator state.
import { Input, Time } from "./types.ts";

type Output = {
  header: { stamp: Time; frame_id: string };
  roll_deg: number;
  pitch_deg: number;
  yaw_deg: number;
  near_gimbal_lock: boolean;
};

// To create a second script for truth, change just these two topic strings.
export const inputs = ["/nautilus/gtsam/odom"];
export const output = "/foxglove_script/gtsam_euler";

export default function script(
  event: Input<"/nautilus/gtsam/odom"> | Input<"/nautilus/gtsam/truth">,
): Output | undefined {
  const q = event.message.pose.pose.orientation;
  const norm = Math.hypot(q.x, q.y, q.z, q.w);
  if (!Number.isFinite(norm) || norm < 1e-12) return undefined;
  const x = q.x / norm;
  const y = q.y / norm;
  const z = q.z / norm;
  const w = q.w / norm;
  const sinPitch = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const degrees = 180 / Math.PI;
  return {
    header: event.message.header,
    roll_deg: degrees * Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch_deg: degrees * Math.asin(sinPitch),
    yaw_deg: degrees * Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    near_gimbal_lock: Math.abs(sinPitch) > 1 - 1e-6,
  };
}
