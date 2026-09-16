# GTSAM backend validation — 2026-09-16

The feature branch includes the ESKF fixes and preserves runtime backend selection.
GTSAM 4.2 was built from `4f66a491ffc83cf092d0d818b11dc35135521612` with
`GTSAM_TANGENT_PREINTEGRATION=OFF` and `GTSAM_BUILD_UNSTABLE=ON`.

| Check | Result |
| --- | --- |
| Pinned GTSAM and gtsam_unstable release build/install | Passed |
| Backend, core and ROS component build/install, GCC 11 / ROS Humble | Passed |
| Core CTest with GTSAM disabled, native GCC 15 | Passed |
| Core and GTSAM CTest with manifold preintegration enabled | 2/2 passed |
| Synthetic ROS contract, ESKF backend | Passed |
| Synthetic ROS contract, GTSAM backend | Passed |
| Ruff, clang-format, ament_cpplint/cppcheck/lint_cmake, whitespace | Passed |

Numerical tests compare preintegrated motion with an independent direct integrator,
first-order bias correction with reintegration, and analytic factor Jacobians with
finite differences. A known aided trajectory crosses multiple fixed-lag windows;
the checks verify covariance cross terms, atomic outlier rejection, marginalization
and revision of earlier retained states by later aiding.

Commands, dependency build options, noise conventions and runtime selection are in
[README.md](README.md). The build helper records the same options used in validation;
its shell syntax was checked, while the dependency build itself was executed directly
with CMake. Testing used a local Docker container rather than cloud runners.

The chosen design reuses the ESKF predictor and ROS adapter (DRY), adds a small
estimator interface (SOLID), and uses the existing GTSAM smoother (KISS). An alternative
is a custom fixed-size C preintegrator and smoother for MCU deployment; that would
avoid host dependencies but requires separate solver, memory and timing validation.
This branch implements the host option first and defers that port (YAGNI).

This is a host prototype. No delayed historical measurement insertion, vehicle
accuracy improvement, real-time performance or MCU suitability is claimed.
