## Joystick interface

A joystick interface for manual control and reference sending of the AUV. Subscribes to Xbox controller inputs and publishes wrench commands (manual mode) or pose references (reference/guidance mode) depending on the active operation mode.

### Launching

```bash
ros2 launch joystick_interface_auv joystick_interface_auv.launch.py
```

#### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `drone` | `nautilus` | Drone model — loads the matching config from `auv_setup/config/robots/<drone>.yaml` |
| `namespace` | `<drone>` | ROS namespace. Defaults to the drone name if left empty |
| `orientation_mode` | `euler` | Reference orientation representation: `euler` (publishes `ReferenceFilter` with RPY angles) or `quat` (publishes `ReferenceFilterQuat` with quaternion) |

The `orientation_mode` must match the reference filter used by the active DP controller. Use `euler` with the adaptive backstepping controller (`reference_filter_dp`) and `quat` with the PID controller (`reference_filter_dp_quat`).

Example — launch with quaternion mode for use with the PID controller:

```bash
ros2 launch joystick_interface_auv joystick_interface_auv.launch.py orientation_mode:=quat
```

### Controller button mapping

| Button | Action |
|---|---|
| **A** | Manual mode (direct wrench from joystick axes) |
| **B** | Toggle software killswitch |
| **X** | Autonomous mode |
| **Y** | Reference mode (joystick incrementally updates the pose reference sent to the DP controller) |

In **reference mode**, the left stick controls surge/sway, triggers control heave, the right stick controls pitch/yaw, and shoulder buttons control roll. Movement is expressed in the body frame and rotated into the world frame before being added to the desired pose.

### Config

Gains for both manual wrench output and reference increments are set in `config/param_joystick_interface_auv.yaml`:

- `joystick_*_gain` — scales raw axis/button input to force/torque in manual mode
- `guidance_*_gain` — scales input to position/orientation increments in reference mode
- `debounce_duration` — minimum seconds between button state changes (prevents double-triggers)
