# Landmark Server

The **Landmark Server** maintains a probabilistic map of detected landmarks using an IPDA filter. It exposes two action interfaces to mission planners: one for **polling** until a landmark is found, and one for **converging** the AUV onto a landmark.

---

## Interfaces

| Interface | Type |
|---|---|
| `LandmarkPolling` | Action server |
| `LandmarkConvergence` | Action server |

---

## LandmarkPolling

Waits until a confirmed track matching the requested `type` and `subtype` is found, then returns the result.

| Field | Type | Description |
|---|---|---|
| `type` | `LandmarkType` | Landmark class to search for |
| `subtype` | `LandmarkSubtype` | Landmark subtype to search for |

---

| Result | Type | Description |
|---|---|---|
| `landmarks` | `LandmarkArray` | All confirmed tracks matching the requested type/subtype |

---

```bash
ros2 action send_goal /orca/landmark_polling vortex_msgs/action/LandmarkPolling "{
  type: {value: 2},
  subtype: {value: 1}
}"
```

---

## LandmarkConvergence

Drives the AUV towards a landmark. Succeeds when the AUV has converged onto the landmark.

| Goal Field | Type | Description |
|---|---|---|
| `type` | `LandmarkType` | Landmark class to converge towards |
| `subtype` | `LandmarkSubtype` | Landmark subtype to converge towards |
| `convergence_offset` | `geometry_msgs/Pose` | Offset from the landmark pose |
| `convergence_threshold` | `float64` | Distance (m) to declare convergence |
| `dead_reckoning_threshold` | `float64` | Distance (m) at which target updates stop|
| `track_loss_timeout_sec` | `float64` | Seconds to wait for landmark to reappear before aborting.|
| `convergence_mode` | `uint8` | Waypoint mode used by the ReferenceFilter (see modes below) |

---

| Result Field | Type | Description |
|---|---|---|
| `success` | `bool` | Whether the AUV successfully converged |
| `landmark_valid` | `bool` | Whether a last-known landmark track exists |
| `landmark` | `Landmark` | Last known pose and covariance of the landmark |

---

```bash
ros2 action send_goal /orca/landmark_convergence vortex_msgs/action/LandmarkConvergence "{
  type: {value: 2},
  subtype: {value: 1},
  convergence_offset: {
    position: {x: 0.0, y: 0.0, z: -1.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  convergence_threshold: 0.3,
  dead_reckoning_threshold: 1.0,
  track_loss_timeout_sec: 10.0,
  convergence_mode: vortex_msgs::msg::Waypoint::ONLY_POSITION,
}"
```

---

## Convergence modes

The `convergence_mode` field controls how the ReferenceFilter will approach the target. It maps to the `Waypoint.mode` constants defined in `vortex_msgs/msg/Waypoint`.

| Value | Constant | Behavior |
|---:|---|---|
| 0 | `FULL_POSE` | Match both position and orientation (full pose control). |
| 1 | `ONLY_POSITION` | Only control position; keep current heading/orientation. |
| 2 | `FORWARD_HEADING` | Drive towards the target while keeping a forward-facing heading |
| 3 | `ONLY_ORIENTATION` | Only control orientation; used when rotating in-place to align with a landmark. |
