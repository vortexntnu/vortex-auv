# Nautilus: GTSAM feedback with Stonefish dynamics

This launch uses the existing Nautilus physical model, quaternion DP controller,
joystick interface, operation-mode manager and QP allocator. **DP receives only
GTSAM estimates**, converted to the body origin. The synthetic trajectory
publisher is not started. Bias estimates remain internal.

```bash
cd /home/vortex/ros2_ws
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch gtsam_navigation stonefish.launch.py
```

Stop previous simulations and the bridge on port 8765 before launching. This
launch starts its own Foxglove bridge; `foxglove:=false` reuses an existing bridge
in the same domain, with `use_sim_time:=false`. `rendering:=false` runs headless.
`input:=keyboard` substitutes the existing keyboard input. `input:=none` is for
automated tests. Only one input publisher is started.

Wait for GTSAM initialization while the vehicle settles on the flat bottom.
The scene starts near the bottom at 10 m depth; it has no current. On an Xbox
controller, **B** toggles killswitch, **A** selects direct manual thrust, and
**Y** captures the current estimated body pose and enters DP reference mode.
Lift away from the floor manually before selecting Y. In reference mode, sticks
move the desired position/orientation; releasing them holds the reference.
The existing X/autonomous button is outside this launch's scope: its commands
are blocked. No automatic square/roll mission is started.

## Data paths

- Native ideal IMU at the CAD mount -> independent noise/bias adapter ->
  `/nautilus/imu/data_raw` -> GTSAM.
- Native DVL -> bottom-validity and 25-degree tilt checks, then velocity noise ->
  `/nautilus/dvl/twist` -> GTSAM. Invalid altitude means no bottom lock; no
  zero-velocity observation is manufactured during loss.
- `/nautilus/gtsam/odom` remains IMU-origin navigation. The controller adapter
  converts position, orientation, twist and covariance to `base_link` and is
  the sole publisher of `/nautilus/pose` and `/nautilus/twist`.
- Manual and DP commands have separate internal topics. A guard selects the
  active mode and publishes `/nautilus/wrench_input`. Killswitch, stale commands,
  missing joystick heartbeat, and stale DP feedback produce zero wrench.
  Manual mode does not require GTSAM availability. There is no truth fallback.
- Stonefish truth goes only to the evaluation node. That node interpolates
  truth at estimate timestamps (maximum bracketing gap 20 ms; no extrapolation)
  and publishes `/nautilus/gtsam/truth`. It never publishes control feedback.

## Foxglove

Connect to `ws://localhost:8765` and select the saved `gtsam estimation` layout.
Existing odometry, truth, metrics, Euler, uncertainty, status and bottom-lock
topics retain their names. In 3D select fixed frame `nautilus/odom_enu` and enable
`/nautilus/gtsam/vehicles`: blue is the estimate, orange is truth, with heading
arrows and bounded trails. The boxes represent the body origin; the estimator
still works at the IMU origin. Enable the robot description to render Nautilus
on its estimated TF. Only GTSAM publishes `odom -> base_link`.

Truth's first matched sample defines its local position and heading. This is a
gauge choice, not fitting truth to the estimated trajectory; roll/pitch errors
are preserved. It approximates the initialization origin within the first output
interval. These plots do not validate absolute position or absolute heading.

## Fidelity and timing limits

- Physics and native IMU target 1000 Hz, DVL 8 Hz, estimated odometry 125 Hz.
  GTSAM retains its five-second lag and 0.1 s maximum keyframe interval.
- The installed Stonefish bridge timestamps with wall time. Every node here
  uses wall time. Run continuously at real-time speed: pause, stepping, speed
  changes and a CPU-overloaded run are not valid estimator timing tests. A pause
  can latch the estimator's IMU-gap fault; restart the whole launch afterward.
- Native IMU/DVL noise is disabled in this dedicated scene. The adapter applies
  the existing provisional STIM300 white noise and bias random walk exactly
  once, and 0.005 m/s independent DVL noise. GTSAM ignores IMU orientation.
- Native IMU uses specific force including gravity subtraction and rigid-body
  lever-arm acceleration. Acceleration range is 98.1 m/s² rather than the
  original scene's 10 m/s² range near gravity.
- Stonefish's DVL is a generic **four-beam** geometric model, not the Nucleus
  three-beam acoustic model. Its native rule can accept a single beam hit even
  beyond 90 degrees of vehicle tilt. The adapter additionally requires the DVL
  downward axis to be within **25 degrees of world down**, for roll and pitch
  together; yaw alone does not affect lock. Override with
  `dvl_max_tilt_deg:=25.0`. At the next 8 Hz ping beyond the cutoff, bottom lock
  becomes false and velocity publication stops. Returning inside the cutoff
  also requires a native bottom hit. This is an assumed operational limit,
  not a verified Nucleus specification or acquisition/hysteresis model.
  Native, noise-free IMU attitude is used only to determine sensor availability,
  with a 20 ms maximum timestamp separation; GTSAM still ignores orientation.
  No estimated attitude or evaluated position error feeds this gate. No claim is made
  about exact Nucleus reacquisition, beam-quality or sound-speed behavior.
- The model, meshes and controller come from other installed workspace packages;
  external repositories are unchanged. The dedicated scene copies the Nautilus
