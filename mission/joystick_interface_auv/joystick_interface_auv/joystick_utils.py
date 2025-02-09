class JoyStates:
    XBOX_MODE = "XBOX_MODE"
    AUTONOMOUS_MODE = "AUTONOMOUS_MODE"
    KILLSWITCH = "KILLSWITCH"
    REFERENCE_MODE = "REFERENCE_MODE"


class Wired:
    joystick_buttons_map = [
        "A",
        "B",
        "X",
        "Y",
        "LB",
        "RB",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
    ]

    joystick_axes_map = [
        "horizontal_axis_left_stick",  # Sway
        "vertical_axis_left_stick",  # Surge
        "LT",  # Negative thrust/torque multiplier
        "horizontal_axis_right_stick",  # Yaw
        "vertical_axis_right_stick",
        "RT",  # Positive thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
    ]


class WirelessXboxSeriesX:
    joystick_buttons_map = [
        "A",
        "B",
        "0",
        "X",
        "Y",
        "0",
        "LB",
        "RB",
        "0",
        "0",
        "back",
        "start",
        "power",
        "stick_button_left",
        "stick_button_right",
        "share_button",
    ]

    joystick_axes_map = [
        "horizontal_axis_left_stick",  # Sway
        "vertical_axis_left_stick",  # Surge
        "horizontal_axis_right_stick",  # Yaw
        "vertical_axis_right_stick",
        "RT",  # Positive thrust/torque multiplier
        "LT",  # Negative thrust/torque multiplier
        "dpad_horizontal",
        "dpad_vertical",
    ]
