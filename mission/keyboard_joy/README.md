# keyboard_joy

`keyboard_joy` is a ROS 2 Python node that publishes `sensor_msgs/Joy` messages based on keyboard input, acting as a simple joystick replacement.
Key-to-axis and key-to-button mappings are configurable via YAML and support both hold and sticky axis modes.

TODO: Currently **only works on Xorg** (not Wayland) due to limitations in global keyboard capture.
