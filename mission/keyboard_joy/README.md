# keyboard_joy

`keyboard_joy` is a ROS 2 Python node that publishes `sensor_msgs/Joy` messages based on keyboard input, acting as a simple joystick replacement.
Key-to-axis and key-to-button mappings are configurable via YAML and support both hold and sticky axis modes.

To install dependencies, run:
```bash
pip install -r requirements.txt
```

### Movement
- **W** – Move forward
- **S** – Move backward
- **A** – Move left
- **D** – Move right
- **Space** – Move up
- **Shift** – Move down

### Rotation
- **Arrow Up** – Pitch up
- **Arrow Down** – Pitch down
- **Arrow Left** – Yaw left
- **Arrow Right** – Yaw right
- **Q** – Roll left
- **E** – Roll right

### Mode Buttons
- **1** – Xbox mode
- **2** – Killswitch
- **3** – Auto mode
- **4** – Reference mode
