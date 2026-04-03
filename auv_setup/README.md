## AUV setup

This package contains global configs and the main launchfiles. The reason for putting these in a separate package is to be able to reach the aforementioned files from anywhere in a simple manner.

### Config
The config folder contains physical parameters related to the AUV and the environment

* environments: Atmosphere, water density, gravity etc..
* robots: Any and all physical constants directly related to the AUV body
* thrusters: Thruster configs for different thruster types

### Launch

This package contains launchfiles for each specific AUV. Additionally `topside.launch.py` is used on the topside computer that the joystick is connected to, for ROV operations.

#### dp.launch.py

Launches the DP (dynamic positioning) stack — a reference filter and a controller — as a composable node container.

```bash
ros2 launch auv_setup dp.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `drone` | `nautilus` | Drone model — loads `auv_setup/config/robots/<drone>.yaml` |
| `namespace` | `<drone>` | ROS namespace. Defaults to the drone name if left empty |
| `controller_type` | `adaptive` | Controller to use: `adaptive` or `pid` |

**`adaptive`** — launches `dp_adapt_backs_controller` with `reference_filter_dp` (Euler/RPY reference filter). Controller params are loaded from `dp_adapt_backs_controller/config/adapt_params_<drone>.yaml`.

```bash
ros2 launch auv_setup dp.launch.py controller_type:=adaptive drone:=nautilus
```

**`pid`** — launches `pid_controller_dp` with `reference_filter_dp_quat` (quaternion reference filter). Controller params are loaded from `pid_controller_dp/config/pid_params.yaml`.

```bash
ros2 launch auv_setup dp.launch.py controller_type:=pid drone:=nautilus
```

> When using the joystick to send references, make sure `orientation_mode` in `joystick_interface_auv` matches the controller: use `euler` with `adaptive` and `quat` with `pid`.

### Description
The description folder contains the URDF and xacro files for the AUVs. The main description launch file is drone_description.launch.py, which makes all static transforms available to the ros graph.
