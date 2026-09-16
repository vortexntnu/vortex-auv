# ESKF fixes validation — 2026-09-16

Base: `origin/dev/eskf_fuse_depth` at `7e13a67d`.

| Check | Environment | Result |
| --- | --- | --- |
| Release core build and CTest | Native GCC 15, Eigen | Passed |
| Core, ROS component and executable build/install | ROS 2 Humble, Ubuntu 22.04, GCC 11 | Passed |
| Release core CTest | ROS container | Passed |
| Synthetic ROS node contract | Actual installed node and rclpy publishers | Passed |
| Python format/import/static checks | Ruff 0.11.4, repository rule selections | Passed |
| C++ format/static checks | clang-format; ament_cpplint/cppcheck with repository options | Passed |
| CMake style and whitespace | ament_lint_cmake; git diff --check | Passed |

The core tests cover sensor Jacobians using independent finite differences,
noise/bias indexing, random-walk consistency, covariance reset, covariance export,
invalid input, outlier rejection, and stationary propagation. The ROS test covers
startup validity, message frames/covariance, outliers, stale output, latched gap
faults and explicit reset recovery. Build and test commands are in [README.md](README.md).

Crabbox was unavailable because the required cloud token was absent. Validation
used a local `ros:humble-ros-base` Docker container and the native compiler.

This implements report findings F01–F08 and supplies regression coverage for F10.
F09's launch configuration is repaired here, but moving Stonefish truth publishers
and validating estimator-controlled simulation require the external simulation
workspace. F11's numerical core is separated from ROS; a bounded-time C/MCU port
is not implemented. No vehicle run, calibrated accuracy comparison, real-time
benchmark or upstream DVL lock/quality validation was performed.
