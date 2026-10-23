// Runtime audit of the checked-in Foxglove script. This deliberately does not
// claim to replace Foxglove's TypeScript compiler or inspect a saved UI layout.
const assert = require("assert");
const fs = require("fs");
const path = require("path");
const vm = require("vm");

let source = fs.readFileSync(path.join(__dirname, "../foxglove/position_metrics.ts"), "utf8");
source = source
  .replace(/import .*?;\n/, "")
  .replace(/\/\/ Types begin[\s\S]*?\/\/ Types end\./, "")
  .replace(/export const /g, "const ")
  .replace(/new Map<[^>]+>\(\)/g, "new Map()")
  .replace("message: Odom", "message")
  .replace("error: number[], covariance: ArrayLike<number>", "error, covariance")
  .replace(/export default function script\([\s\S]*?\): Output \| undefined \{/, "function script(event) {");
const context = vm.createContext({});
vm.runInContext(source + "\nthis.run = script;", context);
const topics = ["/nautilus/gtsam/odom", "/nautilus/gtsam/truth"];
function event(topic, sec, error, valid = true, frame = "odom") {
  const covariance = new Float64Array(36);
  covariance[0] = 2;
  covariance[1] = covariance[6] = 1;
  covariance[7] = 2;
  covariance[14] = valid ? 3 : -1;
  return { topic: topics[topic], message: {
    header: { stamp: { sec, nanosec: 123 }, frame_id: frame },
    child_frame_id: "imu_link",
    pose: { pose: { position: { x: error[0], y: error[1], z: error[2] } }, covariance },
  } };
}
function close(actual, expected) { assert(Math.abs(actual - expected) < 1e-12); }
assert.strictEqual(context.run(event(0, 10, [1, 2, 3])), undefined);
let out = context.run(event(1, 10, [0, 0, 0]));
close(out.position_error_m, Math.sqrt(14));
close(out.rmse_position_m, Math.sqrt(14));
close(out.position_nees, 5); // Full off-diagonal covariance, independent exact result.
assert.strictEqual(context.run(event(1, 11, [0, 0, 0])), undefined);
out = context.run(event(0, 11, [0, 0, 0]));
close(out.position_error_m, 0);
close(out.rmse_position_m, Math.sqrt(7));
assert.strictEqual(out.matched_samples, 2);
assert.strictEqual(context.run(event(0, 11, [10, 10, 10])), undefined);
assert.strictEqual(context.run(event(0, 12, [1, 1, 1], false)), undefined);
assert.strictEqual(context.run(event(1, 12, [0, 0, 0])), undefined);
assert.strictEqual(context.run(event(0, 13, [1, 1, 1])), undefined);
assert.strictEqual(context.run(event(1, 13, [0, 0, 0], true, "other")), undefined);
assert.strictEqual(context.run(event(0, 14, [1, 1, 1])), undefined);
assert.strictEqual(context.run(event(1, 15, [0, 0, 0])), undefined);
// A new simulation epoch must not retain the previous cumulative error.
assert.strictEqual(context.run(event(1, 1, [0, 0, 0])), undefined);
out = context.run(event(0, 1, [0, 0, 0]));
assert.strictEqual(out.matched_samples, 1);
close(out.rmse_position_m, 0);
console.log("PASS: typed arrays, covariance cross terms, RMSE, both arrival orders, duplicates, invalid covariance, frame/stamp mismatch, clock reset");
