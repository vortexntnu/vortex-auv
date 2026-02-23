# line_filtering

Error-state IPDA filter for 2D line tracking.

## Overview

This package tracks 2D lines detected by a sonar (or other sensor) using an
**error-state** formulation inspired by the quaternion error-state EKF used in
`pose_filtering`.

### Why error-state for lines?

A 2D line in Hesse normal form `(rho, theta)` suffers from:

1. **Angle wrapping** – the `0 ↔ 2π` seam makes Gaussian residuals invalid.
2. **Sign ambiguity** – `(rho, theta)` and `(-rho, theta + π)` describe the
   same geometric line, causing `π`-jumps near `rho ≈ 0`.

Instead we keep a **nominal line** `(rho, n)` where `n` is a unit normal and
filter a minimal 2D error state `[δρ, δφ]` in ℝ².

- **Log residual** – sign-align the measurement normal, then compute `δφ` via
  `atan2(cross, dot)`.  No raw angle subtraction.
- **Exp update** – apply `δρ` additively and rotate `n` by `δφ`, then reset.

### Pipeline

1. Subscribe to `LineSegment2DArray` (pixel-space line detections).
2. Subscribe to `SonarInfo` for pixel-to-metric conversion.
3. Convert pixel segments → metric (z = 0) in sonar frame.
4. TF-transform endpoints to the `target_frame` (typically `odom`).
5. Project to 2D (XY plane) and convert to `(rho, n)` measurements.
6. Run the IPDA track manager (gating → update → confirm / delete → create).
7. Publish confirmed tracks as `LineSegment3DArray` and `MarkerArray`.

## Topics

| Direction | Topic                        | Type                              |
|-----------|------------------------------|-----------------------------------|
| Sub       | `/sonar/line_segments`       | `vortex_msgs/LineSegment2DArray`  |
| Sub       | `/sonar/sonar_info`          | `vortex_msgs/SonarInfo`           |
| Pub       | `/filtered_lines`            | `vortex_msgs/LineSegment3DArray`  |
| Pub       | `/filtered_lines_markers`    | `visualization_msgs/MarkerArray`  |

## Launch

```bash
ros2 launch line_filtering line_filtering.launch.py
```
