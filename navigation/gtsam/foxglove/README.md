# Foxglove position metrics

With the standalone simulation connected, open Foxglove's User Scripts sidebar,
create a script, and paste `position_metrics.ts`. Save the script, then subscribe
to `/foxglove_script/gtsam_position_metrics` in a Raw Messages panel to execute it.
These are local Foxglove topics, not ROS publishers. No colcon rebuild is needed.

Add plots for these fields:

- `rmse_position_m`: cumulative 3D position RMSE, in meters.
- `rmse_x_m`, `rmse_y_m`, `rmse_z_m`: cumulative per-axis position RMSE, in meters.
- `position_nees`: instantaneous three-dimensional position NEEDS.
- `nees_expected`, `nees_lower_95`, `nees_upper_95`: reference curves for NEEDS.

For example, use the Y-value
`/foxglove_script/gtsam_position_metrics.position_nees`.
Use Timestamp on the X-axis and Header stamp for each series.
Save and export the whole layout to preserve the script and panels.

The script matches messages by exact header timestamp, accommodates either
arrival order, and requires matching world and child frame IDs. This suits the
checked-in simulator, which publishes truth on the same IMU sampling grid.
Other data sources may require interpolation or an explicit synchronization
tolerance. Unmatched messages are buffered (at most 2000 per topic). Duplicate
timestamps are ignored; backward timestamps clear the statistics. Run only one
estimator and one simulator in the ROS domain. Reconnect when restarting a run
to clear the displayed history as well as the script's statistics.

RMSE is computed over valid matched messages processed by the script, not
necessarily the entire simulation. Check that `matched_samples` increases.
Frames that differ, nonfinite errors, and invalid position covariance matrices
are skipped. Truth is assumed exact, as it is in this synthetic simulation.

Position NEEDS is `e.transpose() * P.inverse() * e`, implemented with a Cholesky
solve. `P` is the full 3x3 world-position covariance block of ROS odometry,
including its cross-covariances; it is not the diagonal alone. This is marginal
position NEEDS, not full 15-state NEEDS.

For an unbiased Gaussian position error with correctly calibrated covariance,
NEEDS has three degrees of freedom and expected value 3. Its approximate central
95% single-sample reference interval is [0.2158, 9.3484]. These are statistical
references, not hard failure thresholds: occasional excursions are expected.
Consecutive samples are correlated, so do not interpret a running average as
an independent-sample chi-square test. Use independent simulation seeds for
consistency evaluation, including the effects of initialization and noise
model assumptions.

DVL NIS is computed internally before gating but is not currently published.
It cannot be reconstructed exactly from the existing odometry topics. Full
state NEEDS and factor-cost diagnostics likewise require estimator-side work.

Foxglove documentation:
https://docs.foxglove.dev/docs/visualization/user-scripts

## Roll, pitch and yaw display

Paste `quaternion_to_euler.ts` into a separate User Script and save it. Add plots
for `/foxglove_script/gtsam_euler.roll_deg`, `.pitch_deg`, and `.yaw_deg`.
To compare truth, duplicate the script and change its `inputs` topic to
`/nautilus/gtsam/truth` and its `output` topic to
`/foxglove_script/gtsam_truth_euler`. Use Header stamp for both sets of series.

The conversion normalizes the quaternion and uses the ZYX convention
`R = Rz(yaw) Ry(pitch) Rx(roll)`. It returns principal display angles: roll/yaw
in [-180, 180] degrees and pitch in [-90, 90]. Quaternion sign changes do not
change these angles. During the continuous `rotate` simulation, Euler angles
wrap and pitch folds; they cannot display the three original unbounded rotation
parameters uniquely. Roll/yaw become ambiguous at pitch +/-90 degrees, indicated
by `near_gimbal_lock`. This does not indicate an estimator failure.
