## Flightstick interface

A flight stick interface for manual control and reference sending of the AUV. Subscribes to Logitech Extreme 3D Pro inputs and publishes wrench commands (manual mode) or pose references (reference/guidance mode) depending on the active operation mode.

This is the Extreme 3D Pro counterpart to `joystick_interface_auv`, which serves the Xbox pad. The two are interchangeable — same topics, same services, same operation modes — so run whichever matches the hardware on the topside.

### Launching

In simulation, select this controller on the sim launch:

```bash
ros2 launch stonefish_sim vortex_sim_launch.py scenario:=tacc controller:=flightstick
```

`controller` defaults to `xbox`, which runs `joystick_interface_auv` instead.

**Do not launch this package separately alongside the sim.** `vortex_sim_launch.py` already starts a joystick interface and a `joy_node` of its own. Running a second interface leaves two nodes subscribed to the same `/joy` and both publishing `wrench_input`, and the Xbox one will read this stick through the wrong map — the throttle becomes yaw, and buttons 3 and 4 become mode switches. Use the `controller` argument instead.

Passing `flightstick` also sets `joy_node`'s global `deadzone` to `0.0`, because this package applies a deadzone per axis — see below. Left at the sim's default of `0.15`, the two would stack.

Running the interface on its own, against a `joy_node` you started yourself:

```bash
ros2 launch flightstick_interface_auv flightstick_interface_auv.launch.py
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
ros2 launch flightstick_interface_auv flightstick_interface_auv.launch.py orientation_mode:=quat
```

The killswitch is always engaged at startup, regardless of what the operation mode manager reports. The AUV is not drivable until button 12 is pressed deliberately.

### Axis mapping

| Axis | Physical control | Action |
|---|---|---|
| 0 | Stick left/right | **Sway** |
| 1 | Stick forward/back | **Surge** |
| 2 | Stick twist | **Yaw** |
| 3 | Throttle lever | Light brightness *(stubbed)* |
| 4 | Hat left/right | *unmapped* |
| 5 | Hat up/down | **Pitch** |

`joy_node` negates every analog axis to reach the ROS convention, so forward, left and up all read positive. Nothing on this stick needs an `invert`.

The throttle lever is absolute and does not self-centre, which suits a dimmer: it reads `-1.0` at the bottom of its travel and `+1.0` at the top, so the bottom of the lever is lights off.

The hat is a digital 8-way switch, not a potentiometer — axes 4 and 5 only ever hold `-1.0`, `0.0` or `+1.0`, and `joy_node` deliberately routes hat axes around its deadzone code.

### Button mapping

| Button | Action |
|---|---|
| **1** (trigger) | Torpedo launch *(stubbed)* |
| **2** (thumb) | Torpedo load/unload *(stubbed)* |
| **3** | Heave down |
| **4** | Heave up |
| **5** | Roll left |
| **6** | Roll right |
| **7** | *unmapped* |
| **8** | Toggle altitude hold |
| **9** | Autonomous mode |
| **10** | Reference mode (joystick incrementally updates the pose reference sent to the DP controller) |
| **11** | Manual mode (direct wrench from joystick axes) |
| **12** | Toggle software killswitch |

Button numbers match the ones moulded into the stick: 1 and 2 on the stick itself, 3–6 on the stick head, 7–12 on the base.

In **reference mode**, movement is expressed in the body frame and rotated into the world frame before being added to the desired pose, exactly as in `joystick_interface_auv`.

**Altitude hold** (button 8) latches the current depth and pins the `z` component of the reference to it, ignoring the heave buttons while active. There is no `HOLD_ALTITUDE` value in `vortex_msgs/OperationMode`, so this is local to the node rather than a fourth operation mode — it only holds depth in reference mode, where a depth setpoint exists. In manual mode it just zeroes the heave buttons.

### Deadzones

`joy_node` exposes a single global `deadzone` applied to every analog axis, which cannot work for this stick: the twist axis rests off-centre and needs a generous band, while the throttle lever driving the lights needs none at all — any deadzone there puts a flat spot in the middle of the dimmer's travel where brightness stops responding.

So run `joy_node` with `deadzone: 0.0` and tune the per-axis values in this package instead:

| Parameter | Default | Axis |
|---|---|---|
| `deadzone_stick_x` | `0.15` | Stick left/right (sway) |
| `deadzone_stick_y` | `0.15` | Stick forward/back (surge) |
| `deadzone_twist` | `0.30` | Stick twist (yaw) |

Each deadzone rescales what is left, so the axis still reaches full `±1.0` at the end of its travel. The hat and the throttle lever never pass through a deadzone.

`deadzone_twist` is the largest of the three because the stick has mechanical crosstalk: pushing sideways for sway puts enough torque on the twist axis to command yaw at the same time. Raise it further if sway still turns the AUV, lower it if yaw feels unresponsive.

### Config

Gains for both manual wrench output and reference increments are set in `config/param_flightstick_interface_auv.yaml`:

- `joystick_*_gain` — scales raw axis/button input to force/torque in manual mode
- `guidance_*_gain` — scales input to position/orientation increments in reference mode
- `debounce_duration` — minimum seconds between button state changes (prevents double-triggers)
- `deadzone_*` — per-axis deadzones, see above
- `light_change_threshold` — minimum brightness change before the stubbed light channel logs again

### Stubbed channels

Neither the torpedo nor the lights are published — both only log, because vortex-auv has no interface for either:

- **Torpedo** — no topic in any robot's `topics:` block, no `vortex_msgs` type, no driver. The only trace is a raw `torpedo.gpio_pin: 15` in `beluga.yaml`. Load/unload and launch fire on the rising edge, and launch refuses unless a torpedo is loaded.
- **Lights** — no topic either. The only trace is `light.pwm_pins` and `pwm.pins.headlights` in `terrapin.yaml`. Brightness is the throttle lever mapped to `0..1`.

To wire either one up for real, add a topic to the robot configs alongside `gripper_servos`, create the publisher in `set_publishers_and_subscribers`, and replace the log call in `handle_torpedo` / `handle_lights`.
