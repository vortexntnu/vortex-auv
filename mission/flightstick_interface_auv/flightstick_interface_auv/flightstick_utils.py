from vortex_msgs.msg import OperationMode


class JoyStates:
    MANUAL_MODE = "manual"
    AUTONOMOUS_MODE = "autonomous"
    KILLSWITCH = "killswitch"
    REFERENCE_MODE = "reference"


modes = {
    OperationMode.AUTONOMOUS: "Autonomous",
    OperationMode.MANUAL: "Manual",
    OperationMode.REFERENCE: "Reference",
}


class LogitechExtreme3DPro:
    """Axis and button layout of the Logitech Extreme 3D Pro flight stick.

    The stick reports 6 axes and 12 buttons. joy_node negates every analog
    axis to reach the ROS convention, so forward, left and up all read
    positive. The hat is a digital 8-way switch: axes 4 and 5 only ever hold
    -1.0, 0.0 or +1.0, and joy_node deliberately skips the deadzone for them.

    Button names match the numbers moulded into the stick itself.
    """

    joystick_buttons_map = [
        "trigger",
        "thumb",
        "button_3",
        "button_4",
        "button_5",
        "button_6",
        "button_7",
        "button_8",
        "button_9",
        "button_10",
        "button_11",
        "button_12",
    ]

    joystick_axes_map = [
        "horizontal_axis_stick",
        "vertical_axis_stick",
        "twist_axis_stick",
        "throttle_lever",
        "hat_horizontal",
        "hat_vertical",
    ]

    num_axes = 6
    num_buttons = 12
