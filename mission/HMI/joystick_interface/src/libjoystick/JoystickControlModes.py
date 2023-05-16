from enum import IntEnum

class JoystickControlModes(IntEnum):
    OPEN_LOOP = 0       # Joystick control
    POSE4_HOLD = 1      # Stationkeep at current x, y, z, psi
    POSE3_HOLD = 2      # stationkeep at current x, y, psi
    DP_CMD = 3          # Give commands to DP at x, y, z, psi
    EMERGENCY_STOP = 4  # Kills DP controller and publishes 0 wrench
    KILLSWITCH = 5      # Kills all operation

# Create a reverse lookup dictionary
JoystickControlModesLookup = {member.value: member.name for member in JoystickControlModes}

# Function to get the variable name from the value
def get_joystick_control_mode_name(value):
    return JoystickControlModesLookup.get(value)